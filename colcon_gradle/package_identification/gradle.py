# Copyright 2018 Esteve Fernandez
# Licensed under the Apache License, Version 2.0

import os
from pathlib import Path
import re

from colcon_core.package_identification \
    import PackageIdentificationExtensionPoint
from colcon_core.plugin_system import satisfies_version


class GradlePackageIdentification(PackageIdentificationExtensionPoint):
    """Identify Gradle packages with `build.gradle` files."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageIdentificationExtensionPoint.EXTENSION_POINT_VERSION,
            '^1.0')

    def identify(self, metadata):  # noqa: D102
        if metadata.type is not None and metadata.type != 'gradle':
            return

        build_gradle = metadata.path / 'build.gradle'
        if not build_gradle.is_file():
            return

        data = extract_data(build_gradle)
        if not data['name'] and not metadata.name:
            raise RuntimeError(
                "Failed to extract project name from '%s'" % build_gradle)

        if metadata.name is not None and metadata.name != data['name']:
            raise RuntimeError('Package name already set to different value')

        metadata.type = 'gradle'
        if metadata.name is None:
            metadata.name = data['name']
        metadata.dependencies['build'] |= data['depends']
        metadata.dependencies['run'] |= data['depends']
        metadata.dependencies['test'] |= data['depends']


def extract_data(build_gradle):
    """
    Extract the project name and dependencies from a build.gradle file.

    :param Path build_gradle: The path of the build.gradle file
    :rtype: dict
    """
    # Content for name
    content_settings_gradle = extract_content(
        build_gradle.parent, 'settings.gradle')

    data = {}
    data['name'] = extract_project_name(content_settings_gradle)
    # fallback to the directory name
    if data['name'] is None:
        data['name'] = build_gradle.parent.name

    # extract dependencies from all Gradle files in the project directory
    data['depends'] = set()

    return data


def extract_content(basepath, filename='build.gradle', exclude=None):
    """
    Get all non-comment lines from build.gradle files under the given basepath.

    :param Path basepath: The path to recursively crawl
    :param list exclude: The paths to exclude
    :rtype: str
    """
    if basepath.is_file():
        content = basepath.read_text(errors='replace')
    elif basepath.is_dir():
        content = ''
        for dirpath, dirnames, filenames in os.walk(str(basepath)):
            # skip sub-directories starting with a dot
            dirnames[:] = filter(lambda d: not d.startswith('.'), dirnames)
            dirnames.sort()

            for name in sorted(filenames):
                if name != filename:
                    continue

                path = Path(dirpath) / name
                if path in (exclude or []):
                    continue

                content += path.read_text(errors='replace') + '\n'
    else:
        return ''
    return _remove_gradle_comments(content)


def _remove_gradle_comments(content):
    # based on https://stackoverflow.com/questions/241327/python-snippet-to-remove-c-and-c-comments#241506  # noqa: E501
    def replacer(match):
        s = match.group(0)
        if s.startswith('/'):
            return ' '  # note: a space and not an empty string
        else:
            return s
    pattern = re.compile(
        r'//.*?$|/\*.*?\*/|\'(?:\\.|[^\\\'])*\'|"(?:\\.|[^\\"])*"',
        re.DOTALL | re.MULTILINE
    )
    return re.sub(pattern, replacer, content)


def extract_project_name(content):
    """
    Extract the Gradle project name from the Gradle settings file.

    Find `rootProject.name` declaration in settings file.

    :param str content: The Gradle build file
    :returns: The project name, otherwise None
    :rtype: str
    """
    # extract project name
    match = re.search(
        # https://regex101.com/r/KzrkzB/1/
        # keyword
        r'rootProject\.name'
        # optional white space
        r'\s*'
        # equal assignment
        '='
        # optional white space
        r'\s*'
        # optional "opening" quote
        '("|\')'
        # project name
        '([a-zA-Z0-9_-]+)'
        # optional "closing" quote (only if an "opening" quote was used)
        r'\1',
        content)
    if not match:
        return None
    return match.group(2)
