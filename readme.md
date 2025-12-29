## How to install
```
apt install python3-venv
python3 -m venv .venv
source .venv/bin/activate
pip install -e . 
```

## build images
``` 
# currently supported image: [mf-mantis-eye]
python3 build_docker.py --image_name mf-mantis-eye
```

## How to run
```
# first of all, you need to modify setup.bash
# MF_DATA_ROOT: root of grain data
# MF_EXTERNAL_DATA_ROOT: root of external data (e.g. rosbag2)

# list all available procedures
mf-eye list-procedures
mf-eye run locate_rosbags
```

# Data Architecture Overview

This project adopts a clear separation between **persistent storage** and **in-memory processing** to ensure scalability, performance, and long-term maintainability when handling large-scale sensor data.

## Design Principles

Persistent storage and query are handled by SQLite.

In-memory synchronization, validation, and real-time processing are handled by Pydantic models.

This separation is intentional and foundational.

- SQLite serves as the source of truth for recorded data.
- Pydantic models represent short-lived, computation-oriented data structures used during runtime.

## Rationale

Large-scale sensor systems often suffer from performance and maintenance issues when storage formats and runtime data models are tightly coupled.  
To avoid this, the system enforces a strict boundary:

- **SQLite**
  - Long-lived data persistence
  - Efficient querying, filtering, and aggregation
  - Referential integrity between entities (e.g., images and bounding boxes)
  - Reproducibility and offline analysis

- **Pydantic**
  - In-memory data validation
  - Synchronization across heterogeneous sensor streams
  - Real-time and near-real-time processing
  - Algorithm-facing, type-safe representations

## Architectural Implications

- Pydantic models do not manage disk persistence directly.
- Database schemas are optimized for query and storage, not for algorithmic convenience.
- Explicit adapter or repository layers handle conversions between SQLite rows and Pydantic models.
- Changes in storage schema do not directly impact real-time processing logic, and vice versa.

## Summary

SQLite defines **what exists and what was recorded**.

Pydantic defines **what is being processed right now**.

Maintaining this boundary allows the system to scale from real-time pipelines to offline analytics without accumulating structural debt.
****