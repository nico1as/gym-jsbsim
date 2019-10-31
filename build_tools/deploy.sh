#!/bin/bash
if [[ "$TRAVIS_OS_NAME" != "linux" ]]; then
  python3 setup.py bdist_wheel || python setup.py bdist_wheel
  twine upload --skip-existing --username "galleon" --password "$PASSWORD" dist/*.whl
else
  twine upload --skip-existing --username "galleon" --password "$PASSWORD" dist/*manylinux*.whl
fi
