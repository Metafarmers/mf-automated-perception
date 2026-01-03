# MF Automated Perception  
## Core Architectural Invariants (STABLE)

Version: 1.0  
Purpose:  
This document defines non-negotiable architectural invariants of the GP Graph framework.  
Anything not written here is allowed to change.

---

## 1. Core Model

### 1.1 Definitions (FIXED)

- Grain  
  Immutable data artifact with provenance and sealed schema.

- Procedure  
  Pure transformation from input Grains to exactly one output Grain.

- GP Graph  
  Directed graph where Grains are nodes and Procedures are edges.

These meanings MUST NOT be reinterpreted.

---

## 2. Layer Boundaries (DO NOT CROSS)

There are exactly three layers:

1. Grain Layer  
   - Data storage  
   - Schema  
   - Provenance  

2. Procedure Layer  
   - Transformation logic only  

3. Execution Layer  
   - Docker  
   - Runners  
   - Workflow orchestration  

No layer may reference concepts from a lower layer.

---

## 3. Grain Invariants

### 3.1 Identity

- Every Grain has:
  - key: Tuple[str, ...]
  - uuid: 8-character hex string
- Filesystem path equals Grain key
- Class name equals PascalCase of key
- UUID collision MUST be checked before creation

---

### 3.2 Lifecycle

Grain lifecycle is strictly:

NEW → PROVENANCE_SET → CREATED → OPEN → CREATED

Violating this order MUST raise an error.

---

### 3.3 Provenance

- Provenance MUST be set before creation
- Provenance is immutable after creation
- Provenance records:
  - source_procedure
  - source_grain_keys
  - creator
  - workflow_uuid
  - git_commit
  - created_at

Grains MUST NOT infer or modify provenance.

---

### 3.4 Schema

- Schemas are applied and sealed at creation time
- Sealed schema files are copied into the grain directory
- Loaded grains MUST use sealed schemas, not global ones
- Schema meaning is defined by:
  - schema file names
  - schema content hash
- Schema mutation after creation is forbidden

---

### 3.5 Grain Responsibilities

Grains are passive.

They MUST NOT:
- Execute procedures
- Know about Docker or workflows
- Contain orchestration logic

They MAY:
- Store data
- Manage schema
- Provide read/write DB access
- Support inspection

---

## 4. Procedure Invariants

### 4.1 Identity

- Procedure key equals directory name
- Procedure ID = {key}:{version}
- One Procedure produces exactly one output Grain

---

### 4.2 Contract

Procedures MUST:
- Declare required and optional input grain keys
- Declare exactly one output grain key
- Be independent of execution order

Procedures MUST NOT:
- Create output grains
- Set provenance
- Depend on Docker or host environment

---

### 4.3 Execution Semantics

- Input grains are loaded by the framework
- Output grain is created by the framework
- Procedure logic operates only inside _run()

Procedures are pure transformations.

---

## 5. Execution Invariants

### 5.1 Container Boundary

- Host and container communicate ONLY via:
  - Filesystem (grains)
  - Environment variables
  - CLI arguments
  - Result JSON

No shared state and no IPC shortcuts.

---

### 5.2 Result Contract

- Every execution produces exactly one ProcedureRunResult
- Result is written unconditionally (success or failure)
- Result is the only execution truth source

---

## 6. Factories and Discovery

- Grains and Procedures are discovered by convention
- No manual registration
- File name, class name, and key MUST match

Violation MUST raise an error at load time.

---

## 7. What This Document Does NOT Specify

The following are explicitly allowed to change:

- WorkflowRunner design
- GUI or ViewModel structure
- Storage backend
- Inspection APIs
- Schema evolution strategy

If it is not written here, it is not frozen.

---

## 8. Design Philosophy

- Prefer visibility over prevention
- Prefer sealed artifacts over migrations
- Prefer explicit boundaries over clever abstractions
- Prefer reproducibility over convenience

---

## Final Rule

If a future change violates any rule in this document,  
the change is wrong, not the document.
