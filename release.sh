#!/bin/bash -e
# Username is 'micropsi-industries'
# Get the password from doik

# Find tag
TAG="$(git describe --tags)"
echo "Tag is '${TAG}'"
[[ -n "${TAG}" ]] || echo "Git tag must be checked out."

# Find or make venv
PYTHON=".venv/bin/python"
[[ -f "${PYTHON}" ]] || python3.6 -m venv .venv --prompt $(basename $(pwd))
echo "Python is '${PYTHON}'"

# update build tools
${PYTHON} -m pip install -U pip setuptools wheel

# update twine
${PYTHON} -m pip install -U twine

# Check module version matches
VERSION=$(${PYTHON} -c "exec(open('micropsi_integration_sdk/version.py').read()); print(VERSION)")
echo "Version is '${VERSION}'"
[[ "${TAG}" == "${VERSION}" ]] || echo "Git tag must match version."

# Build source distribution
${PYTHON} setup.py sdist
echo "build complete"

# Upload to pypi
${PYTHON} -m twine upload "dist/micropsi-integration-sdk-${TAG}.tar.gz"
echo "upload complete"
