# Makefile for robocore-cli development and testing

.PHONY: help install test test-unit test-integration test-e2e test-local test-vm test-sbc clean lint format

# Default target
help:
	@echo "RoboCore CLI Development Commands"
	@echo "================================="
	@echo ""
	@echo "Setup Commands:"
	@echo "  install          Install robocore-cli in development mode"
	@echo "  install-dev      Install with all development dependencies"
	@echo ""
	@echo "Testing Commands:"
	@echo "  test             Run all tests (unit + integration + e2e-local)"
	@echo "  test-unit        Run unit tests only"
	@echo "  test-integration Run integration tests only"
	@echo "  test-e2e         Run all end-to-end tests"
	@echo "  test-local       Run local single-host e2e tests"
	@echo "  test-vm          Run multi-VM tests with Vagrant"
	@echo "  test-sbc         Run tests on SBC farm"
	@echo "  test-performance Run performance benchmarks"
	@echo ""
	@echo "VM Management:"
	@echo "  vm-up            Start all test VMs"
	@echo "  vm-down          Stop all test VMs"
	@echo "  vm-clean         Destroy all test VMs"
	@echo "  vm-ssh           SSH into manager VM"
	@echo ""
	@echo "Code Quality:"
	@echo "  lint             Run linting checks"
	@echo "  format           Format code with black"
	@echo "  validate         Validate test configurations"
	@echo ""
	@echo "Cleanup:"
	@echo "  clean            Clean build artifacts and test containers"
	@echo "  clean-images     Remove test Docker images"

# Installation targets
install:
	pip install -e .

install-dev:
	pip install -e . --break-system-packages
	pip install -r tests/requirements-test.txt --break-system-packages
	pip install black flake8 pytest pytest-cov pytest-asyncio --break-system-packages

# Testing targets
test: test-unit test-integration test-local

test-unit:
	@echo "Running unit tests..."
	pytest tests/unit/ -v --cov=core --cov=commands --cov-report=html

test-integration:
	@echo "Running integration tests..."
	pytest tests/integration/ -v

test-e2e: test-local test-vm

test-local:
	@echo "Running local end-to-end tests..."
	docker network create robocore-test || true
	pytest tests/e2e/test_single_host.py -v
	docker network rm robocore-test || true

test-vm:
	@echo "Running multi-VM tests..."
	cd tests && vagrant up
	cd tests && vagrant ssh test-runner -c "cd /robocore-cli && python -m pytest tests/e2e/test_multi_host.py -v"
	cd tests && vagrant halt

test-sbc:
	@echo "Running SBC farm tests..."
	python tests/e2e/run_sbc_farm_test.py --config tests/e2e/sbc_farm_config.yaml --scenario regression

test-performance:
	@echo "Running performance benchmarks..."
	python tests/performance/benchmark_suite.py --output benchmark_results.json

# VM management
vm-up:
	cd tests && vagrant up

vm-down:
	cd tests && vagrant halt

vm-clean:
	cd tests && vagrant destroy -f

vm-ssh:
	cd tests && vagrant ssh manager

vm-status:
	cd tests && vagrant status

# Code quality
lint:
	@echo "Running linting checks..."
	flake8 core/ commands/ cli.py --max-line-length=88 --extend-ignore=E203,W503
	black --check core/ commands/ cli.py

format:
	@echo "Formatting code..."
	black core/ commands/ cli.py

validate:
	@echo "Validating test configurations..."
	python -c "import yaml; [yaml.safe_load(open(f)) for f in ['tests/e2e/configs/single-host.yaml', 'tests/e2e/configs/multi-host.yaml', 'tests/e2e/configs/cross-arch.yaml']]"
	@echo "All configurations are valid ✓"

# Cleanup targets
clean:
	@echo "Cleaning build artifacts..."
	rm -rf build/ dist/ *.egg-info/
	rm -rf .pytest_cache/ .coverage htmlcov/
	rm -rf tests/e2e/test-projects/*/
	docker system prune -f || true

clean-images:
	@echo "Removing test Docker images..."
	docker images --filter "reference=localhost:5000/test_*" -q | xargs -r docker rmi -f
	docker images --filter "reference=*test*" -q | xargs -r docker rmi -f

# Development helpers
dev-setup: install-dev
	@echo "Setting up development environment..."
	pre-commit install || echo "pre-commit not available, skipping hook setup"
	@echo "Development environment ready!"

# CI simulation (runs the same tests as GitHub Actions)
ci-test:
	@echo "Running CI test suite locally..."
	make test-unit
	make test-integration
	make test-local
	make lint

# Documentation generation
docs:
	@echo "Generating documentation..."
	# Add documentation generation here when ready

# Container registry for testing
registry-start:
	docker run -d -p 5000:5000 --restart=always --name test-registry registry:2 || true

registry-stop:
	docker stop test-registry || true
	docker rm test-registry || true

# Quick project setup for testing
test-project:
	@echo "Creating test project..."
	rm -rf test-example-project
	mkdir test-example-project
	cd test-example-project && robocore-cli . init --non-interactive
	cp tests/e2e/configs/single-host.yaml test-example-project/config.yaml
	cp -r tests/e2e/fixtures/test-packages test-example-project/packages
	@echo "Test project created in test-example-project/"

# Version and release helpers
version:
	@python -c "import yaml; print('robocore-cli version:', yaml.safe_load(open('setup.py').read())['version'])"

# Advanced testing scenarios
test-cross-platform:
	@echo "Testing cross-platform builds..."
	docker buildx create --name multiarch --use || true
	docker buildx inspect --bootstrap
	cd test-example-project && robocore-cli . stage --platform linux/amd64,linux/arm64

test-stress:
	@echo "Running stress tests..."
	python tests/stress/stress_test.py --components 10 --hosts 5 --duration 10m

# SBC farm management
sbc-setup:
	@echo "Setting up SBC farm..."
	ansible-playbook tests/e2e/sbc_farm/setup.yml -i tests/e2e/sbc_farm/inventory.ini

sbc-status:
	@echo "Checking SBC farm status..."
	python tests/e2e/sbc_farm/check_status.py

# Help for specific areas
help-testing:
	@echo "Testing Help"
	@echo "============"
	@echo ""
	@echo "Test Types:"
	@echo "  unit         - Fast tests for individual functions/classes"
	@echo "  integration  - Tests for component interactions"
	@echo "  e2e-local    - Full workflow tests on single machine"
	@echo "  e2e-vm       - Multi-host tests using Vagrant VMs"
	@echo "  e2e-sbc      - Real hardware tests on SBC farm"
	@echo ""
	@echo "Quick Start:"
	@echo "  make install-dev     # Setup development environment"
	@echo "  make test-project    # Create test project"
	@echo "  make test-local      # Run basic e2e tests"
	@echo "  make vm-up           # Start VMs for multi-host testing"
	@echo "  make test-vm         # Run multi-host tests"

help-sbc:
	@echo "SBC Farm Help"
	@echo "============="
	@echo ""
	@echo "Setup your SBC farm following: tests/e2e/sbc_farm_setup.md"
	@echo ""
	@echo "Commands:"
	@echo "  make sbc-setup   # Provision SBCs with Ansible"
	@echo "  make sbc-status  # Check farm health"
	@echo "  make test-sbc    # Run tests on real hardware"