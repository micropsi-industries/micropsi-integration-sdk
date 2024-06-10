#!/bin/bash -e
# Username is 'micropsi-industries'
# Get the password from doik

# Make sure git status is all clear.
if output=$(git status --porcelain) && [[ -n "$output" ]]; then
    echo "Git checkout must be clean."
    echo "${output}"
    exit 1
fi

# Find tag
TAG="$(git describe --tags)"

# Find or make venv
PYTHON=".venv/bin/python"
[[ -f "${PYTHON}" ]] || python3 -m venv .venv --prompt $(basename $(pwd))
echo "Python is '${PYTHON}'"

# update build tools
${PYTHON} -m pip install -U pip setuptools wheel

# update twine
${PYTHON} -m pip install -U twine

# Check module version matches
VERSION=$(${PYTHON} -c "exec(open('micropsi_integration_sdk/version.py').read()); print(VERSION)")
echo "Version is '${VERSION}'"
[[ "${TAG}" == "${VERSION}" ]] || { echo "Git tag '${TAG}' must match micropsi_integration_sdk.__version__ '${VERSION}'."; exit ${ERRCODE}; }

# Build source distribution
${PYTHON} setup.py sdist
echo "build complete"

# Upload to pypi
${PYTHON} -m twine upload "dist/micropsi-integration-sdk-${TAG}.tar.gz"
echo "Upload complete"
echo "Go to https://github.com/micropsi-industries/micropsi-integration-sdk and make a release from the '${TAG}' tag."
