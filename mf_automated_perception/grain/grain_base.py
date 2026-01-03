import sqlite3
import uuid
from abc import ABC
from datetime import datetime
from enum import Enum, auto
from pathlib import Path
from typing import Any, ClassVar, Iterable, List, Optional, Tuple

from pydantic import BaseModel, PrivateAttr

from mf_automated_perception.env import (
  BASE_SCHEMA_ROOT,
  GIT_COMMIT_HASH,
  GRAIN_DATA_ROOT,
)
from mf_automated_perception.grain.grain_manifest import GrainManifest
from mf_automated_perception.utils.time_utils import _KST, now

GrainKey = Tuple[str, ...]

class GrainState(Enum):
  NEW = auto()
  PROVENANCE_SET = auto()
  CREATED = auto()
  OPEN = auto()


class GrainBase(ABC, BaseModel):
  # ===============================================================
  # identity (class-level metadata)
  # ===============================================================

  # NOTE:
  # Use Any to avoid pydantic v2 ClassVar evaluation issues.
  # Actual type is GrainKey and validated in model_post_init.
  key: ClassVar[Any]

  SCHEMA_FILES: Tuple[str, ...]

  # ===============================================================
  # private state
  # ===============================================================

  _manifest: Optional[GrainManifest] = PrivateAttr(default=None)
  _conn: Optional[sqlite3.Connection] = PrivateAttr(default=None)
  _is_created: bool = PrivateAttr(default=False)
  _grain_id: Optional[str] = PrivateAttr(default=None)
  _uuid: Optional[str] = PrivateAttr(default=None)
  _state: GrainState = PrivateAttr(default=GrainState.NEW)

  # ===============================================================
  # pydantic hook
  # ===============================================================

  def model_post_init(self, __context: Any) -> None:
    # validate class-level key once
    k = self.__class__.key

    if not isinstance(k, tuple):
      raise TypeError(
        f"{self.__class__.__name__}.key must be tuple[str, ...], got {type(k)}"
      )
    if not k:
      raise ValueError(
        f"{self.__class__.__name__}.key must not be empty"
      )
    if not all(isinstance(x, str) for x in k):
      raise TypeError(
        f"{self.__class__.__name__}.key elements must be str, got {k}"
      )

  # ===============================================================
  # derived paths
  # ===============================================================

  def _require_state(
    self,
    *,
    allowed: tuple["GrainState", ...],
    action: str,
  ) -> None:
    if self._state not in allowed:
      raise RuntimeError(
        f"{action}() not allowed in state {self._state.name}, "
        f"allowed states: {[s.name for s in allowed]}"
      )


  @property
  def grain_data_root(self) -> Path:
    return Path(GRAIN_DATA_ROOT)

  @property
  def grain_id(self) -> str:
    if self._grain_id is None:
      if self._manifest is None:
        raise RuntimeError("grain_id is not available before manifest is set")
      if self._uuid is None:
        raise RuntimeError("grain_id is not available before UUID is set")

      self._grain_id = (
        f"{self._manifest.created_at.astimezone(_KST).strftime('%Y-%m-%d_%H-%M-%S')}_"
        f"{self._uuid}"
      )

    return self._grain_id


  @property
  def uuid(self) -> str:
    """
    Stable unique identifier for this grain.
    Generated exactly once during creation.
    """
    if self._uuid is None:
      raise RuntimeError("Grain UUID is not available before creation")
    return self._uuid



  def _check_uuid_collision(self) -> None:
    """Check if any grain with same UUID already exists in grain_data_root."""
    from mf_automated_perception.env import GRAIN_DATA_ROOT

    target_uuid = self.uuid
    key_path = GRAIN_DATA_ROOT / Path(*self.key)

    if not key_path.exists():
      return  # No grains of this type exist yet

    # Search all grain directories for matching UUID
    for grain_dir in key_path.iterdir():
      if grain_dir.is_dir() and grain_dir.name.endswith(f"_{target_uuid}"):
        raise RuntimeError(
          f"UUID collision detected: {target_uuid}. "
          f"Existing grain: {grain_dir}"
        )



  # rel path starts from (grain_data_dir), which is defined by env vars both in host and docker container
  @property
  def grain_data_dir_abs(self) -> Path:
    return self.grain_data_root.joinpath(*self.key) / self.grain_id

  @property
  def grain_data_dir_rel(self) -> Path:
    return Path(*self.key) / self.grain_id

  @property
  def db_path_abs(self) -> Path:
    return self.grain_data_dir_abs / "data.db3"

  @property
  def db_path_rel(self) -> Path:
    return self.grain_data_dir_rel / "data.db3"

  @property
  def manifest_path_abs(self) -> Path:
    return self.grain_data_dir_abs / "manifest.json"

  @property
  def manifest_path_rel(self) -> Path:
    return self.grain_data_dir_rel / "manifest.json"


  @property
  def schema_dir_abs(self) -> Path:
    return self.grain_data_dir_abs / "schema"

  @property
  def schema_dir_rel(self) -> Path:
    return self.grain_data_dir_rel / "schema"

  @property
  def is_created(self) -> bool:
    return self._is_created
  # ===============================================================
  # manifest access (read-only)
  # ===============================================================

  @property
  def manifest(self) -> GrainManifest:
    if self._manifest is None:
      raise RuntimeError("Manifest has not been initialized")
    return self._manifest



  # ===============================================================
  # provenance API
  # ===============================================================

  def set_provenance(
    self,
    *,
    source_procedure: str,
    source_grain_keys: Iterable[GrainKey],
    creator: str,
    workflow_uuid: Optional[str] = None,
    created_at: Optional[datetime] = None,
  ) -> None:
    self._require_state(
      allowed=(GrainState.NEW,),
      action="set_provenance",
    )

    if self._is_created:
      raise RuntimeError(
        "set_provenance() must not be called after grain creation"
      )

    if not source_procedure:
      raise ValueError("source_procedure must not be empty")
    if not creator:
      raise ValueError("creator must not be empty")

    self._manifest = GrainManifest(
      key=list(self.key),
      git_commit=GIT_COMMIT_HASH or "unknown",
      source_procedure=source_procedure,
      source_grain_keys=[list(k) for k in source_grain_keys],
      creator=creator,
      workflow_uuid=workflow_uuid,
      created_at=created_at or now(),
      schema_files=list(self.SCHEMA_FILES),
    )

    self._state = GrainState.PROVENANCE_SET


  # ===============================================================
  # lifecycle
  # ===============================================================

  def create(
    self,
    *,
    grain_uuid: Optional[str] = None,
  ) -> None:
    """Create grain directory and initialize database."""
    self._require_state(
      allowed=(GrainState.PROVENANCE_SET,),
      action="create",
    )

    if self._manifest is None:
      raise RuntimeError(
        "create() requires provenance to be set via set_provenance()"
      )

    # 1. UUID 결정
    if self._uuid is None:
      if grain_uuid is not None:
        self._uuid = grain_uuid
      else:
        self._uuid = uuid.uuid4().hex[:8]

    # 2. collision check
    self._check_uuid_collision()

    # 3. materialize
    self.grain_data_dir_abs.mkdir(parents=True, exist_ok=True)
    sqlite3.connect(self.db_path_abs).close()

    self.manifest_path_abs.write_text(
      self._manifest.model_dump_json(indent=2)
    )

    self._is_created = True

    self._state = GrainState.CREATED
    self._initialize_schema()


  def _initialize_schema(self) -> None:
    if not self.SCHEMA_FILES:
      return

    schema_dir = self.schema_dir_abs
    schema_dir.mkdir(parents=True, exist_ok=True)

    conn = self.open()
    try:
      for name in self.SCHEMA_FILES:
        src = BASE_SCHEMA_ROOT / name
        if not src.exists():
          raise FileNotFoundError(f"Schema file not found: {src}")

        sql = src.read_text()

        # 1. DB에 적용
        conn.executescript(sql)

        # 2. grain 내부에 schema 원본 저장
        dst = schema_dir / name
        dst.write_text(sql)

      conn.commit()
    finally:
      self.close()


  # ===============================================================
  # database access
  # ===============================================================
  def open(self) -> sqlite3.Connection:
    self._require_state(
      allowed=(GrainState.CREATED, GrainState.OPEN),
      action="open",
    )
    if self._conn is None:
      self._conn = sqlite3.connect(self.db_path_abs)
      self._conn.row_factory = sqlite3.Row
    return self._conn

  def close(self) -> None:
    if self._conn is not None:
      self._conn.close()
      self._conn = None
      self._state = GrainState.CREATED


  # ===============================================================
  # inspection
  # ===============================================================

  def print_grain_summary(self) -> None:
    print("\n[Grain Summary]")
    if self._manifest is None:
      print("Manifest not initialized.")
    else:
      print(self._manifest.model_dump())

  # ===============================================================
  # deletion
  # ===============================================================

  def delete(self, *, force: bool = False) -> None:
    """
    Delete this grain instance from disk.

    Rules:
      - Grain must be created.
      - Database connection must be closed.
      - By default, basic sanity checks are enforced.
      - force=True bypasses sanity checks.
    """
    if not self._is_created:
      raise RuntimeError(
        "delete() can only be called on a created grain"
      )

    if self._conn is not None:
      raise RuntimeError(
        "delete() requires database connection to be closed"
      )

    grain_dir = self.grain_data_dir_abs

    if not grain_dir.exists():
      raise FileNotFoundError(
        f"Grain directory not found: {grain_dir}"
      )

    if not force:
      # minimal safety checks
      if not self.manifest_path_abs.exists():
        raise RuntimeError(
          f"manifest.json not found in grain directory: {grain_dir}"
        )
      if not self.db_path_abs.exists():
        raise RuntimeError(
          f"data.db3 not found in grain directory: {grain_dir}"
        )

    # recursive delete
    for p in grain_dir.rglob("*"):
      if p.is_file() or p.is_symlink():
        p.unlink()
    for p in sorted(grain_dir.rglob("*"), reverse=True):
      if p.is_dir():
        p.rmdir()
    grain_dir.rmdir()

    # reset internal state
    self._manifest = None
    self._is_created = False


  # ===============================================================
  # loading
  # ===============================================================

  @classmethod
  def load_from_dir(
    cls,
    *,
    grain_data_dir: Path,
  ) -> "GrainBase":
    # must be called on subclass
    if cls is GrainBase:
      raise RuntimeError(
        "GrainBase.load_from_dir() must be called on a concrete subclass"
      )

    grain_data_dir = Path(grain_data_dir)
    manifest_path = grain_data_dir / "manifest.json"
    db_path = grain_data_dir / "data.db3"
    schema_dir = grain_data_dir / "schema"

    # basic file checks
    if not manifest_path.exists():
      print(f"loading {grain_data_dir} failed - manifest.json missing")
      return None
    if not db_path.exists():
      print(f"loading {grain_data_dir} failed - data.db3 missing")
      return None

    manifest = GrainManifest.model_validate_json(
      manifest_path.read_text()
    )

    # key must match class
    if tuple(manifest.key) != cls.key:
      raise ValueError(
        f"Grain key mismatch: "
        f"class key={cls.key}, manifest key={tuple(manifest.key)}"
      )

    # validate sealed schema
    if manifest.schema_files:
      if not schema_dir.exists():
        print(f"Loading {grain_data_dir} failed - Schema directory missing")
        return None

      missing = [
        name for name in manifest.schema_files
        if not (schema_dir / name).exists()
      ]
      if missing:
        print(f"Loading {grain_data_dir} failed - Schema files missing: {missing}")
        return None

    # restore grain state
    grain = cls()
    grain._manifest = manifest
    grain._is_created = True
    grain._grain_id = grain_data_dir.name
    grain._uuid = grain_data_dir.name.split("_")[-1]

    # prevent schema re-application
    grain.SCHEMA_FILES = tuple(manifest.schema_files)
    grain._state = GrainState.CREATED
    return grain


  def list_tables(self) -> List[str]:
    if not self._is_created:
      raise RuntimeError("list_tables() requires created grain")

    conn = sqlite3.connect(self.db_path_abs)
    try:
      rows = conn.execute(
        "SELECT name FROM sqlite_master WHERE type='table';"
      ).fetchall()
      return [r[0] for r in rows]
    finally:
      conn.close()

  def get_schema_sql(self) -> str:
    """
    Return concatenated SQL schema used to create this grain.
    """
    if not self._is_created:
      raise RuntimeError("get_schema_sql() requires created grain")

    schema_dir = self.schema_dir_abs
    print('schema_dir: ', schema_dir)
    if not schema_dir.exists():
      return ""

    parts: List[str] = []

    for path in sorted(schema_dir.glob("*.sql")):
      parts.append(
        f"-- schema file: {path.name}\n{path.read_text()}"
      )

    return "\n\n".join(parts)

  def get_schema_files(self) -> List[str]:
    if not self._is_created:
      raise RuntimeError("get_schema_files() requires created grain")

    schema_dir = self.schema_dir_abs
    if not schema_dir.exists():
      return []

    return sorted(p.name for p in schema_dir.glob("*.sql"))

  def table_view(
    self,
    *,
    table: str,
    limit: int = 10,
  ):
    if not self._is_created:
      raise RuntimeError("table_view() requires created grain")

    self.open()
    try:
      return self._conn.execute(
        f"SELECT * FROM {table} LIMIT ?;",
        (limit,),
      ).fetchall()
    finally:
      self.close()

  def dict_view(
    self,
    *,
    table: str,
    limit: int = 10,
  ):
    if not self._is_created:
      raise RuntimeError("dict_view() requires created grain")

    rows = self.table_view(table=table, limit=limit)
    return [dict(row) for row in rows]

  def summarize_db(
    self,
    *,
    tables: Optional[Iterable[str]] = None,
    max_rows: int = 5,
  ) -> None:
    self._require_state(
      allowed=(GrainState.CREATED, GrainState.OPEN),
      action="summarize_db",
    )

    if tables is None:
      tables = self.list_tables()

    for table in tables:
      print("=" * 80)
      print(f"TABLE: {table}")

      rows = self.dict_view(table=table, limit=max_rows)

      if not rows:
        print("(empty)")
        continue

      # header
      keys = list(rows[0].keys())
      header = " | ".join(keys)
      print(header)
      print("-" * len(header))

      # rows
      for row in rows:
        values = [str(row[k]) for k in keys]
        print(" | ".join(values))

      # row count
      total = self.count_rows(table)
      if total > max_rows:
        print(f"... ({total - max_rows} more rows)")

  def count_rows(self, table: str) -> int:
    conn = sqlite3.connect(self.db_path_abs)
    try:
      cur = conn.execute(f"SELECT COUNT(*) FROM {table}")
      return cur.fetchone()[0]
    finally:
      conn.close()

  def list_column_names(
    self,
    *,
    table: str,
  ) -> list[str]:
    if not self._is_created:
      raise RuntimeError("list_column_names() requires created grain")

    conn = sqlite3.connect(self.db_path_abs)
    conn.row_factory = sqlite3.Row
    try:
      rows = conn.execute(
        f"PRAGMA table_info({table});"
      ).fetchall()

      if not rows:
        raise ValueError(f"{table} is not a valid table or view")

      return [row["name"] for row in rows]
    finally:
      conn.close()
