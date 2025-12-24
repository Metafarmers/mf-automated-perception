from pydantic import BaseModel, PrivateAttr
from pathlib import Path
from typing import Tuple, Dict, Any, List, Optional, Iterable, ClassVar
from datetime import datetime, timezone, timedelta
from abc import ABC
import sqlite3
import json
import os
import yaml

from mf_automated_perception.env import DATA_ROOT, SCHEMA_ROOT

GrainKey = Tuple[str, ...]
KST = timezone(timedelta(hours=9))

class GrainBase(ABC, BaseModel):
  # ---------- identity ----------
  key: ClassVar[GrainKey]

  # ---------- provenance (public, read-only by convention) ----------
  source_procedure: str = ""
  source_grain_keys: List[GrainKey] = []
  created_at: datetime = datetime.now(timezone.utc)
  creator: str = ""

  # ---------- schema declaration (override in subclasses) ----------
  SCHEMA_FILES: ClassVar[List[str]] = []

  # ---------- private internal state ----------
  _conn: Optional[sqlite3.Connection] = PrivateAttr(default=None)
  _is_created: bool = PrivateAttr(default=False)

  # ===============================================================
  # initialization
  # ===============================================================

  def model_post_init(self, __context: Any) -> None:
    pass

  # ===============================================================
  # read-only public interface
  # ===============================================================

  @property
  def grain_data_root(self) -> Path:
    return Path(DATA_ROOT) / "grain_data"
  @property
  def grain_lib_root(self) -> Path:
    return Path(DATA_ROOT) / "grain_lib"
  @property
  def log_root(self) -> Path:
    return self.grain_data_root / "logs"

  # ===============================================================
  # derived paths
  # ===============================================================

  @property
  def grain_data_dir(self) -> Path:
    return self.grain_data_root.joinpath(*self.key) / self.grain_id

  @property
  def grain_id(self) -> str:
    return self.created_at.astimezone(KST).strftime("%Y-%m-%d_%H-%M-%S")

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
  # lifecycle
  # ===============================================================

  def create(self) -> None:
    if not self.source_procedure:
      raise RuntimeError(
        "Grain.create() is not allowed before provenance is set. "
      )
    if not self.creator:
      raise RuntimeError(
        "Grain.create() requires creator to be set."
      )

    self.grain_data_dir.mkdir(parents=True, exist_ok=True)
    sqlite3.connect(self.db_path).close()
    self._create_manifest()
    self._initialize_schema()
    self._is_created = True


  def _initialize_schema(self) -> None:
    if not self.SCHEMA_FILES:
      return

    conn = self.open()
    try:
      schema_dir = self._schema_dir()
      for name in self.SCHEMA_FILES:
        sql = (schema_dir / name).read_text()
        conn.executescript(sql)
      conn.commit()
    finally:
      self.close()

  def _schema_dir(self) -> Path:
    return SCHEMA_ROOT

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
  # manifest
  # ===============================================================

  def _create_manifest(self) -> None:
    manifest: Dict[str, Any] = {
      "key": list(self.key),
      "source_procedure": self.source_procedure,
      "source_grain_keys": [list(k) for k in self.source_grain_keys],
      "created_at": self.created_at.isoformat(),
      "creator": self.creator,
    }
    self.manifest_path.write_text(json.dumps(manifest, indent=2))

  # ===============================================================
  # provenance (internal)
  # ===============================================================

  def set_provenance(
    self,
    *,
    source_procedure: str,
    source_grain_keys: Iterable[GrainKey],
    creator: str,
    created_at: Optional[datetime] = None,
  ) -> None:
    if not source_procedure:
      raise ValueError("source_procedure must not be empty")
    if not creator:
      raise ValueError("creator must not be empty")

    self.source_procedure = source_procedure
    self.source_grain_keys = list(source_grain_keys)
    self.creator = creator
    self.created_at = created_at or datetime.now(timezone.utc)

  def print_grain_summary(self) -> None:
    """
    Print frequently used GrainBase attributes for debugging and inspection.
    """
    print("\n[Grain Summary]")
    print(f"  key              : {self.key}")
    print(f"  grain_data_root  : {self.grain_data_root}")
    print(f"  grain_lib_root   : {self.grain_lib_root}")
    print(f"  grain_data_dir   : {self.grain_data_dir}")
    print(f"  grain_id         : {self.grain_id}")
    print(f"  db_path          : {self.db_path}")
    print(f"  manifest_path    : {self.manifest_path}")
    print(f"  source_procedure : {self.source_procedure}")
    print(f"  source_grain_keys: {self.source_grain_keys}")
    print(f"  creator          : {self.creator}")
    print(f"  created_at       : {self.created_at.isoformat()}")

    if self._conn is not None:
      print("  db_connection    : OPEN")
    else:
      print("  db_connection    : CLOSED")

  def list_tables(self) -> List[str]:
    conn = self.open()
    rows = conn.execute(
      "SELECT name FROM sqlite_master WHERE type='table';"
    ).fetchall()
    return [r[0] for r in rows]

  def preview_table(
    self,
    *,
    table: str,
    limit: int = 10,
  ) -> List[tuple]:
    conn = self.open()
    return conn.execute(
      f"SELECT * FROM {table} LIMIT {limit};"
    ).fetchall()

  def summarize_db(
    self,
    *,
    max_rows: int = 10,
    logger=None,
  ) -> None:
    logger = logger or print

    tables = self.list_tables()
    logger(f"[DB Summary] tables={tables}")

    for t in tables:
      rows = self.preview_table(table=t, limit=max_rows)
      logger(f"[Table: {t}] showing {len(rows)} rows")
      for r in rows:
        logger(f"  {r}")

  @classmethod
  def load_from_dir(
    cls,
    *,
    grain_data_dir: Path,
  ) -> "GrainBase":
    """
    Load an already-created grain from its directory.

    Must be called on a concrete Grain subclass, not GrainBase.
    """

    # ---- guard: abstract base class ----
    if cls is GrainBase:
      raise RuntimeError(
        "GrainBase.load_from_dir() must not be called directly. "
        "Use a concrete Grain subclass."
      )

    grain_data_dir = Path(grain_data_dir)
    manifest_path = grain_data_dir / "manifest.json"
    db_path = grain_data_dir / "data.db3"

    if not manifest_path.exists():
      raise FileNotFoundError(f"manifest.json not found: {manifest_path}")

    if not db_path.exists():
      raise FileNotFoundError(f"data.db3 not found: {db_path}")

    # ---- read manifest ----
    manifest = json.loads(manifest_path.read_text())

    # ---- validate key ----
    manifest_key = tuple(manifest["key"])
    if manifest_key != cls.key:
      raise ValueError(
        f"Grain key mismatch: "
        f"class key={cls.key}, manifest key={manifest_key}"
      )

    # ---- instantiate grain (bind config via model_post_init) ----
    grain = cls()

    # ---- restore provenance via canonical API ----
    grain.set_provenance(
      source_procedure=manifest.get("source_procedure", ""),
      source_grain_keys=[
        tuple(k) for k in manifest.get("source_grain_keys", [])
      ],
      creator=manifest.get("creator", ""),
      created_at=datetime.fromisoformat(manifest["created_at"]),
    )

    # ---- sanity check: directory consistency ----
    expected_dir = grain.grain_data_root
    if expected_dir.resolve() != grain.grain_data_root.resolve():
      raise ValueError(
        "grain_data_root mismatch:\n"
        f"  expected: {expected_dir}\n"
        f"  actual  : {grain.grain_data_root}"
      )

    # ---- mark as created ----
    grain._is_created = True

    return grain
