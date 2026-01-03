# GP Graph Framework: Quick Reference

## Architecture at a Glance

```
┌─────────────────────────────────────────────────────────────┐
│                     GP GRAPH SYSTEM                         │
│                                                             │
│  ┌──────────────┐        ┌──────────────┐                  │
│  │    GRAIN     │───────>│  PROCEDURE   │                  │
│  │   (Node)     │        │    (Edge)    │                  │
│  └──────────────┘        └──────────────┘                  │
│         │                        │                          │
│         │                        │                          │
│    [Data Layer]            [Transform Layer]                │
│                                                             │
│  ┌──────────────────────────────────────────────────┐      │
│  │         EXECUTION LAYER                          │      │
│  │  (Docker Runtime, Orchestration, Workflows)      │      │
│  └──────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

## The Three Layers (NEVER MIX THESE)

### 1. Grain Layer (Passive Data)
- **What:** SQLite database + provenance + schema
- **Knows About:** Nothing (pure data container)
- **Does:** Store data, track lineage
- **Cannot:** Execute procedures, orchestrate workflows, know about Docker

### 2. Procedure Layer (Transformation Logic)
- **What:** `input_grains → [transformation] → output_grain`
- **Knows About:** Input/output grain schemas, transformation algorithm
- **Does:** Read from input grains, write to output grain
- **Cannot:** Create grains, set provenance, assume execution order

### 3. Execution Layer (Orchestration)
- **What:** Docker runner, workflow engine, CLI
- **Knows About:** How to run procedures, where grains live
- **Does:** Load grains, run procedures in Docker, collect results
- **Cannot:** Know about specific schemas, contain transformation logic

---

## Grain Quick Reference

### Creating a Grain

```python
from mf_automated_perception.grain.defs.my_grain import MyGrain

grain = MyGrain()

# 1. Set provenance (REQUIRED before create)
grain.set_provenance(
    source_procedure="my_proc:1.0.0",
    source_grain_keys=[("input", "grain", "key")],
    creator="researcher_name",
    workflow_uuid="optional-uuid",  # Optional
)

# 2. Create grain (auto-generates UUID or use custom)
grain.create()  # Auto UUID
# OR
grain.create(grain_uuid="deadbeef")  # Custom UUID

# 3. Write data
conn = grain.open()
try:
    conn.execute("INSERT INTO my_table VALUES (?)", (value,))
    conn.commit()
finally:
    grain.close()
```

### Loading a Grain

```python
from mf_automated_perception.grain.grain_factory import GrainFactory

# Load latest grain of a type
grain = GrainFactory.load_grain(
    key=("my", "grain", "key"),
    load_rule="latest",
    grain_data_root=GRAIN_DATA_ROOT,
)

# Load specific grain by UUID
grain = GrainFactory.load_grain(
    key=("my", "grain", "key"),
    load_rule="uuid",
    uuid="deadbeef",
    grain_data_root=GRAIN_DATA_ROOT,
)
```

### Grain File Structure

```
{GRAIN_DATA_ROOT}/
  my/grain/key/
    2026-01-03_12-34-56_deadbeef/
      manifest.json         # Provenance metadata
      data.db3              # SQLite database
      schema/               # Sealed schema files
        001_init_v1.sql
        301_odometry_v1.sql
```

---

## Procedure Quick Reference

### Defining a Procedure

```python
# File: procedure/defs/my_proc/definition.py
from typing import ClassVar, Optional, Tuple, Type
from pydantic import BaseModel
from mf_automated_perception.grain.grain_base import GrainKey
from mf_automated_perception.procedure.core.procedure_base import ProcedureBase

class MyProcConfig(BaseModel):
    param1: str
    param2: int = 42

class MyProc(ProcedureBase):
    key: ClassVar[str] = "my_proc"
    version: ClassVar[str] = "0.0.1"
    docker_image: ClassVar[str] = "my-image"
    docker_image_tag: ClassVar[str] = "latest"

    ParamModel: ClassVar[Optional[Type[BaseModel]]] = MyProcConfig

    required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = (
        ("input", "grain"),
    )
    optional_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
    output_grain_key: ClassVar[GrainKey] = ("output", "grain")
```

### Implementing a Procedure

```python
# File: procedure/defs/my_proc/implementation.py
from typing import Dict, Optional
from mf_automated_perception.grain.grain_base import GrainBase, GrainKey
from .definition import MyProc

class MyProcImpl(MyProc):
    def _run(
        self,
        *,
        input_grains: Optional[Dict[GrainKey, GrainBase]],
        output_grain: Optional[GrainBase],
        config,  # Instance of MyProcConfig
        logger,
    ) -> None:
        logger.info("Starting procedure")

        # Access input
        input_grain = input_grains[("input", "grain")]
        conn_in = input_grain.open()
        rows = conn_in.execute("SELECT * FROM my_table").fetchall()
        input_grain.close()

        # Process and write output
        conn_out = output_grain.open()
        try:
            for row in rows:
                processed = self._process(row, config)
                conn_out.execute("INSERT INTO ...", processed)
            conn_out.commit()
        finally:
            output_grain.close()

        logger.info("Procedure complete")

    def _process(self, row, config):
        # Your transformation logic here
        return processed_data
```

---

## Schema Quick Reference

### Schema Naming Convention

```
{category}_{name}_v{version}.sql

Category Ranges:
  001-099: Core/initialization
  101-199: Sensor data (images, point clouds)
  201-299: Perception/ML (YOLO, segmentation)
  301-399: Motion/pose (odometry, IMU)
  901-999: Application-specific
```

### Common Schemas

```python
# Minimal grain (just initialization)
SCHEMA_FILES = ("001_init_v1.sql",)

# Odometry grain
SCHEMA_FILES = (
    "001_init_v1.sql",
    "301_odometry_v1.sql",
)

# Image + YOLO detections
SCHEMA_FILES = (
    "001_init_v1.sql",
    "101_image_v1.sql",
    "201_yolo_bbox_v1.sql",
)

# Rosbag path (custom)
SCHEMA_FILES = (
    "001_init_v1.sql",
    "901_path_to_raw_rosbag_v1.sql",
)
```

### Schema Immutability Rules

- ✅ Reuse existing schemas
- ✅ Create new schemas in 900+ range for custom needs
- ❌ NEVER modify existing core schemas (001-399)
- ❌ NEVER change schema after grain creation

---

## Container & Execution Reference

### Environment Variables (Required)

```bash
# On host
export MF_GRAIN_DATA_ROOT=/path/to/grain/data
export MF_EXTERNAL_DATA_ROOT=/path/to/external/data
export MF_PROJECT_ROOT=/path/to/mf-automated-perception
export MF_BASE_SCHEMA_ROOT=$MF_PROJECT_ROOT/mf_automated_perception/grain/schema

# Inside container (automatically set by docker_runner)
MF_GRAIN_DATA_ROOT=/data
MF_EXTERNAL_DATA_ROOT=/external_data
MF_PROJECT_ROOT=/workspace
MF_BASE_SCHEMA_ROOT=/workspace/mf_automated_perception/grain/schema
```

### Docker Volume Mounts (Standard)

```
Host Path                     → Container Path
{MF_PROJECT_ROOT}             → /workspace
{MF_GRAIN_DATA_ROOT}          → /data
{MF_EXTERNAL_DATA_ROOT}       → /external_data
```

### Running a Procedure

```python
from mf_automated_perception.host.runtime.docker_runner import run_procedure

result = run_procedure(
    procedure="my_proc",
    docker_image="my-image:latest",
    creator="researcher_name",
    params_file=Path("params/my_proc.yaml"),  # Optional
    bash=False,  # Set True for interactive shell
)

print(result.status)        # "success" or "failed"
print(result.exit_code)     # 0 or error code
print(result.duration_sec)  # Execution time
```

---

## Naming Conventions Cheat Sheet

### Grains
| Component | Convention | Example |
|-----------|------------|---------|
| Module file | `snake_case` (key joined with `_`) | `raw_rosbag_path.py` |
| Class name | `PascalCase` (key words capitalized) | `RawRosbagPath` |
| Class key | `Tuple[str, ...]` | `("raw", "rosbag", "path")` |
| Directory | Nested by key components | `grain_data_root/raw/rosbag/path/` |

### Procedures
| Component | Convention | Example |
|-----------|------------|---------|
| Module dir | `snake_case` | `locate_rosbags/` |
| Class name | `PascalCase` | `LocateRosbags` |
| Impl class | `{Class}Impl` | `LocateRosbagsImpl` |
| Key | `str` (matches dir) | `"locate_rosbags"` |
| Version | Semver string | `"0.0.1"` |

---

## Common Patterns

### Pattern 1: No-Input Procedure (Data Collector)

```python
class DataCollector(ProcedureBase):
    required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = ()
    output_grain_key: ClassVar[GrainKey] = ("raw", "data")

class DataCollectorImpl(DataCollector):
    def _run(self, *, input_grains, output_grain, config, logger):
        # input_grains will be None or {}
        # Collect data from external source
        # Write to output_grain
```

### Pattern 2: Single-Input Transformation

```python
class Transformer(ProcedureBase):
    required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = (
        ("input", "type"),
    )
    output_grain_key: ClassVar[GrainKey] = ("output", "type")

class TransformerImpl(Transformer):
    def _run(self, *, input_grains, output_grain, config, logger):
        input_grain = input_grains[("input", "type")]
        # Read from input_grain, transform, write to output_grain
```

### Pattern 3: Multi-Input Fusion

```python
class Fuser(ProcedureBase):
    required_input_grain_keys: ClassVar[Tuple[GrainKey, ...]] = (
        ("sensor", "a"),
        ("sensor", "b"),
    )
    output_grain_key: ClassVar[GrainKey] = ("fused", "data")

class FuserImpl(Fuser):
    def _run(self, *, input_grains, output_grain, config, logger):
        grain_a = input_grains[("sensor", "a")]
        grain_b = input_grains[("sensor", "b")]
        # Fuse data from both inputs
```

---

## File System Layout Reference

```
mf-automated-perception/
├── mf_automated_perception/
│   ├── grain/
│   │   ├── defs/              # One grain per file
│   │   │   ├── raw_rosbag_path.py
│   │   │   ├── slam3d_glim.py
│   │   │   └── _dummy.py      # Underscore = test grain
│   │   ├── schema/            # Shared SQL schemas
│   │   │   ├── 001_init_v1.sql
│   │   │   ├── 101_image_v1.sql
│   │   │   └── 301_odometry_v1.sql
│   │   ├── grain_base.py
│   │   ├── grain_factory.py
│   │   └── grain_manifest.py
│   │
│   ├── procedure/
│   │   ├── core/
│   │   │   ├── procedure_base.py
│   │   │   ├── procedure_factory.py
│   │   │   └── entrypoint.py
│   │   └── defs/              # One procedure per directory
│   │       ├── locate_rosbags/
│   │       │   ├── definition.py
│   │       │   ├── implementation.py
│   │       │   └── __init__.py
│   │       └── offline_glim/
│   │           ├── definition.py
│   │           ├── implementation.py
│   │           └── __init__.py
│   │
│   ├── host/
│   │   ├── runtime/
│   │   │   └── docker_runner.py
│   │   └── cli/
│   │       └── main.py
│   │
│   ├── runner/
│   │   └── gp_runner.py       # In-process runner (no Docker)
│   │
│   ├── utils/
│   │   ├── gp_logger.py
│   │   └── time_utils.py
│   │
│   └── env.py                 # Environment validation
│
├── tests/
│   ├── grain/
│   │   └── test_grain.py
│   └── procedure/
│       └── test_procedure.py
│
└── params/                    # Procedure config files
    ├── locate_rosbags.yaml
    └── offline_glim.yaml
```

---

## State Machine Reference

### Grain State Machine

```
┌─────┐  set_provenance()  ┌──────────────┐  create()  ┌─────────┐
│ NEW │ ──────────────────> │ PROVENANCE_  │ ────────> │ CREATED │
└─────┘                     │     SET      │            └─────────┘
                            └──────────────┘                 │
                                                             │ open()
                                                             v
                            ┌──────────────┐            ┌─────────┐
                            │ CREATED      │ <───────── │  OPEN   │
                            │ (after close)│   close()  └─────────┘
                            └──────────────┘
```

**State Validation:**
- `set_provenance()`: Only in `NEW` state
- `create()`: Only in `PROVENANCE_SET` state
- `open()`: Only in `CREATED` or `OPEN` state
- `close()`: Transitions from `OPEN` to `CREATED`

---

## Checklist for Adding New Components

### New Grain Checklist
- [ ] Create file: `grain/defs/{key_joined_with_underscores}.py`
- [ ] Class name matches key in PascalCase
- [ ] Declare `key: ClassVar[GrainKey]` matching module name
- [ ] Declare `SCHEMA_FILES: Tuple[str, ...]`
- [ ] All schemas exist in `grain/schema/`
- [ ] Write test in `tests/grain/`
- [ ] Do NOT modify `grain_base.py`, `grain_factory.py`

### New Procedure Checklist
- [ ] Create directory: `procedure/defs/{procedure_key}/`
- [ ] Create `definition.py` with metadata-only class
- [ ] Create `implementation.py` with `{Class}Impl` class
- [ ] Declare all required ClassVars (key, version, docker_image, etc.)
- [ ] Implement `_run()` method in implementation class
- [ ] Create config YAML in `params/{procedure_key}.yaml` if needed
- [ ] Write test in `tests/procedure/`
- [ ] Do NOT modify `procedure_base.py`, `procedure_factory.py`, `entrypoint.py`

### New Schema Checklist
- [ ] Use appropriate category number range
- [ ] Follow naming: `{category}_{name}_v{version}.sql`
- [ ] Place in `grain/schema/`
- [ ] If category < 900: Treat as core (reusable, immutable)
- [ ] If category >= 900: Application-specific (more flexible)
- [ ] Test with a grain that uses it

---

## Common Errors & Solutions

### Error: "Grain key mismatch"
**Cause:** Module name, class name, or `key` ClassVar don't match
**Solution:** Ensure consistency:
```python
# File: grain/defs/my_grain_name.py
class MyGrainName(GrainBase):  # PascalCase of file name
    key: ClassVar[GrainKey] = ("my", "grain", "name")  # Matches file name
```

### Error: "UUID collision detected"
**Cause:** Trying to create grain with UUID that already exists
**Solution:** Delete existing grain or use different UUID

### Error: "manifest.json not found"
**Cause:** Grain directory incomplete or corrupted
**Solution:** Ensure `set_provenance()` and `create()` completed successfully

### Error: "input_grains must be provided"
**Cause:** Procedure declares required inputs but received None
**Solution:** Check that input grains exist and are loaded correctly

### Error: "MF_GRAIN_DATA_ROOT is not set"
**Cause:** Missing environment variable
**Solution:** Run `source setup.bash` or set manually

### Error: "Definition classes must not implement runtime logic"
**Cause:** `_run()` method exists in definition.py
**Solution:** Move `_run()` to implementation.py only

---

**For full details, see [ARCHITECTURE_RULES.md](./ARCHITECTURE_RULES.md)**
