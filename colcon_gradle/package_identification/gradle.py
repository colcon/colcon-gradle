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

        metadata.type = 'gradle'
        if metadata.name is None:
            metadata.name = data['name']


def extract_data(build_gradle):
    """
    Extract the project name and dependencies from a build.gradle file.

    :param Path build_gradle: The path of the build.gradle file
    :rtype: dict
    """

    data = {}
    data['name'] = build_gradle.parent.name

    return data
