# Define variables
PROJECT_NAME := route_planner.go
BIN_DIR := bin
SRC_FILES := $(shell find . -type f -name "*.go")

# Default target
.PHONY: all
all: build

# Build target
.PHONY: build
build:
	@echo "Building $(PROJECT_NAME)..."
	go build -o $(BIN_DIR)/$(PROJECT_NAME) ./$(PROJECT_NAME) # Adjust path as needed

# Run target
.PHONY: run
run: build
	@echo "Running $(PROJECT_NAME)..."
	./$(BIN_DIR)/$(PROJECT_NAME) ../data/nodes.csv ../data/edges.csv 1 6 bellman-ford # Adjust arguments as needed
	./$(BIN_DIR)/$(PROJECT_NAME) ../data/nodes.csv ../data/edges.csv 1 6 astar # Adjust arguments as needed
	./$(BIN_DIR)/$(PROJECT_NAME) ../data/nodes.csv ../data/edges.csv 1 6 dijkstra # Adjust arguments as needed


# Test target
.PHONY: test
test:
	@echo "Running tests..."
	go test ./...

# Clean target
.PHONY: clean
clean:
	@echo "Cleaning build artifacts..."
	rm -rf $(BIN_DIR)

# Format target
.PHONY: fmt
fmt:
	@echo "Formatting Go code..."
	gofmt -w $(SRC_FILES)

# Help target
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  all     - Build the project (default)"
	@echo "  build   - Compile the Go application"
	@echo "  run     - Build and run the Go application"
	@echo "  test    - Run all tests"
	@echo "  clean   - Remove compiled binaries and build artifacts"
	@echo "  fmt     - Format Go source code"
	@echo "  help    - Display this help message"