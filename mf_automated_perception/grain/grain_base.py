import sqlite3
import uuid
from abc import ABC
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, ClassVar, Iterable, List, Optional, Tuple

from pydantic import BaseModel, PrivateAttr

from mf_automated_perception.env import GIT_COMMIT_HASH, GRAIN_DATA_ROOT, SCHEMA_ROOT
from mf_automated_perception.grain.grain_manifest import GrainManifest

GrainKey = Tuple[str, ...]
KST = timezone(timedelta(hours=9))


class GrainBase(ABC, BaseModel):
  # ===============================================================
  # identity (class-level metadata)
  # ===============================================================

  # NOTE:
  # Use Any to avoid pydantic v2 ClassVar evaluation issues.
  # Actual type is GrainKey and validated in model_post_init.
  key: ClassVar[Any]

  SCHEMA_FILES: ClassVar[List[str]] = []

  # ===============================================================
  # private state
  # ===============================================================

  _manifest: Optional[GrainManifest] = PrivateAttr(default=None)
  _conn: Optional[sqlite3.Connection] = PrivateAttr(default=None)
  _is_created: bool = PrivateAttr(default=False)
  _grain_id: Optional[str] = PrivateAttr(default=None)

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

  @property
  def grain_data_root(self) -> Path:
    return Path(GRAIN_DATA_ROOT)

  @property
  def grain_id(self) -> str:
    if self._grain_id is None:
      if self._manifest is None:
        raise RuntimeError("grain_id is not available before manifest is set")

      self._grain_id = (
        f"{self._manifest.created_at.astimezone(KST).strftime('%Y-%m-%d_%H-%M-%S')}_"
        f"{uuid.uuid4().hex[:8]}"
      )

    return self._grain_id


  @property
  def grain_data_dir(self) -> Path:
    return self.grain_data_root.joinpath(*self.key) / self.grain_id

  @property
  def db_path(self) -> Path:
    return self.grain_data_dir / "data.db3"

  @property
  def manifest_path(self) -> Path:
    return self.grain_data_dir / "manifest.json"

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
    created_at: Optional[datetime] = None,
  ) -> None:
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
      created_at=created_at or datetime.now(timezone.utc),
      schema_files=list(self.SCHEMA_FILES),
    )


  # ===============================================================
  # lifecycle
  # ===============================================================

  def create(self) -> None:
    if self._manifest is None:
      raise RuntimeError(
        "create() requires provenance to be set via set_provenance()"
      )

    grain_dir = self.grain_data_dir
    grain_dir.mkdir(parents=True, exist_ok=True)

    db_path = grain_dir / "data.db3"
    sqlite3.connect(db_path).close()

    self.manifest_path.write_text(
      self._manifest.model_dump_json(indent=2)
    )

    self._initialize_schema()
    self._is_created = True
    assert self._grain_id is not None


  def _initialize_schema(self) -> None:
    if not self.SCHEMA_FILES:
      return

    conn = self.open()
    try:
      for name in self.SCHEMA_FILES:
        sql = (SCHEMA_ROOT / name).read_text()
        conn.executescript(sql)
      conn.commit()
    finally:
      self.close()

  # ===============================================================
  # database access
  # ===============================================================

  def open(self) -> sqlite3.Connection:
    if self._conn is None:
      self._conn = sqlite3.connect(self.db_path)
    return self._conn

  def close(self) -> None:
    if self._conn is not None:
      self._conn.close()
      self._conn = None

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

    grain_dir = self.grain_data_dir

    if not grain_dir.exists():
      raise FileNotFoundError(
        f"Grain directory not found: {grain_dir}"
      )

    if not force:
      # minimal safety checks
      if not self.manifest_path.exists():
        raise RuntimeError(
          f"manifest.json not found in grain directory: {grain_dir}"
        )
      if not self.db_path.exists():
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
    if cls is GrainBase:
      raise RuntimeError(
        "GrainBase.load_from_dir() must be called on a concrete subclass"
      )

    grain_data_dir = Path(grain_data_dir)
    manifest_path = grain_data_dir / "manifest.json"
    db_path = grain_data_dir / "data.db3"

    if not manifest_path.exists():
      raise FileNotFoundError(f"manifest.json not found: {manifest_path}")
    if not db_path.exists():
      raise FileNotFoundError(f"data.db3 not found: {db_path}")

    manifest = GrainManifest.model_validate_json(
      manifest_path.read_text()
    )

    if tuple(manifest.key) != cls.key:
      raise ValueError(
        f"Grain key mismatch: "
        f"class key={cls.key}, manifest key={tuple(manifest.key)}"
      )

    grain = cls()
    grain._manifest = manifest
    grain._is_created = True
    grain._grain_id = grain_data_dir.name
    return grain

  def list_tables(self) -> List[str]:
    if not self._is_created:
      raise RuntimeError("list_tables() requires created grain")

    conn = sqlite3.connect(self.db_path)
    try:
      rows = conn.execute(
        "SELECT name FROM sqlite_master WHERE type='table';"
      ).fetchall()
      return [r[0] for r in rows]
    finally:
      conn.close()

  def table_view(
    self,
    *,
    table: str,
    limit: int = 10,
  ):
    if not self._is_created:
      raise RuntimeError("table_view() requires created grain")

    print('db path: ', self.db_path)
    conn = sqlite3.connect(self.db_path)
    conn.row_factory = sqlite3.Row
    try:
      return conn.execute(
        f"SELECT * FROM {table} LIMIT ?;",
        (limit,),
      ).fetchall()
    finally:
      conn.close()

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
    if not self._is_created:
      raise RuntimeError("summarize_db() requires created grain")

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
    conn = sqlite3.connect(self.db_path)
    try:
      cur = conn.execute(f"SELECT COUNT(*) FROM {table}")
      return cur.fetchone()[0]
    finally:
      conn.close()

