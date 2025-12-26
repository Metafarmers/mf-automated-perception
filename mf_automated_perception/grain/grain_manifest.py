from datetime import datetime
from typing import List

from pydantic import BaseModel


class GrainManifest(BaseModel):
  # identity
  key: List[str]

  # provenance (minimal)
  git_commit: str
  schema_files: List[str] = []


  # creation info
  source_procedure: str
  source_grain_keys: List[List[str]]
  creator: str
  created_at: datetime
