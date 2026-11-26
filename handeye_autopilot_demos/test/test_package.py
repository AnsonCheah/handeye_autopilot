"""Version consistency across the three packages (03-milestones M0)."""

import pathlib
import re
import xml.etree.ElementTree as ET

REPO = pathlib.Path(__file__).resolve().parents[2]
PACKAGES = ['handeye_autopilot', 'handeye_autopilot_interfaces', 'handeye_autopilot_demos']


def _xml_version(pkg):
    return ET.parse(REPO / pkg / 'package.xml').getroot().findtext('version')


def test_versions_match_everywhere():
    versions = {pkg: _xml_version(pkg) for pkg in PACKAGES}
    for pkg in PACKAGES:
        setup = REPO / pkg / 'setup.py'
        if setup.exists():
            m = re.search(r"version='([^']+)'", setup.read_text())
            versions[f'{pkg}/setup.py'] = m and m.group(1)
    assert len(set(versions.values())) == 1, versions
    assert re.fullmatch(r'\d+\.\d+\.\d+', next(iter(versions.values())))


def test_every_package_is_apache_and_has_changelog():
    for pkg in PACKAGES:
        root = ET.parse(REPO / pkg / 'package.xml').getroot()
        assert root.findtext('license') == 'Apache-2.0', pkg
        assert root.find('maintainer').get('email'), pkg
        assert (REPO / pkg / 'CHANGELOG.rst').exists(), pkg
