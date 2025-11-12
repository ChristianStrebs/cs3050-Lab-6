# Makefile for route_planner.py
# Usage:
#   make            # run with defaults
#   make run        # same as above
#   make run START=10 END=20 NODES=path/nodes.csv EDGES=path/edges.csv
#   make venv       # create a local .venv (optional)
#   make run-venv   # run using .venv's python
#   make help       # show targets

SHELL := /bin/bash

# ---- Config (override any of these on the command line) ---------------------
PYTHON ?= python3
SCRIPT ?= route_planner.py

DATA_DIR ?= ../data
NODES ?= $(DATA_DIR)/nodes.csv
EDGES ?= $(DATA_DIR)/edges.csv
START ?= 1
END   ?= 3
ALGO  ?= bellman-ford
ALGO2  ?= dijkstra
ALGO3  ?= astar

ARGS := $(NODES) $(EDGES) $(START) $(END) $(ALGO)
ARGS2 := $(NODES) $(EDGES) $(START) $(END) $(ALGO2)
ARGS3 := $(NODES) $(EDGES) $(START) $(END) $(ALGO3)

	
# Virtualenv directory (optional)
VENV_DIR := .venv
VENV_PY  := $(VENV_DIR)/bin/python

# ---- Phony targets ----------------------------------------------------------
.PHONY: run run-venv venv check clean help

# Default target
run: check ## Run the route planner with defaults (override START/END/NODES/EDGES)
	@echo "▶ Running: $(PYTHON) $(SCRIPT) $(ARGS)"
	@$(PYTHON) $(SCRIPT) $(ARGS)
	@$(PYTHON) $(SCRIPT) $(ARGS2)
	@$(PYTHON) $(SCRIPT) $(ARGS3)

run-venv: check ## Run using the virtualenv's Python
	@test -x "$(VENV_PY)" || { echo "No $(VENV_PY). Run 'make venv' first."; exit 1; }
	@echo "▶ Running (venv): $(VENV_PY) $(SCRIPT) $(ARGS)"
	@$(VENV_PY) $(SCRIPT) $(ARGS)

venv: ## Create a local virtual environment (.venv) and upgrade pip
	@$(PYTHON) -m venv $(VENV_DIR)
	@source $(VENV_DIR)/bin/activate && python -m pip install --upgrade pip
	@echo "✔ Created $(VENV_DIR). Use 'make run-venv' to run with it."

check: ## Verify required files exist
	@test -f "$(SCRIPT)" || { echo "Missing script: $(SCRIPT)"; exit 1; }
	@test -f "$(NODES)"  || { echo "Missing nodes CSV: $(NODES)"; exit 1; }
	@test -f "$(EDGES)"  || { echo "Missing edges CSV: $(EDGES)"; exit 1; }

clean: ## Remove Python cache files and build artifacts
	@find . -name '__pycache__' -type d -prune -exec rm -rf {} +
	@find . -name '*.pyc' -delete -o -name '*.pyo' -delete
	@echo "✔ Cleaned python caches."

help: ## Show this help
	@echo "Makefile targets:"
	@grep -E '^[a-zA-Z0-9_-]+:.*?## ' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-12s\033[0m %s\n", $$1, $$2}'