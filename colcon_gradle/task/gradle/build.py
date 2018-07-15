# Copyright 2018 Esteve Fernandez
# Licensed under the Apache License, Version 2.0

import ast
import os
from pathlib import Path
import re

from colcon_gradle.task.gradle import GRADLE_EXECUTABLE
from colcon_core.environment import create_environment_scripts
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.shell import get_command_environment
from colcon_core.task import check_call
from colcon_core.task import TaskExtensionPoint

logger = colcon_logger.getChild(__name__)


class GradleBuildTask(TaskExtensionPoint):
    """Build gradle packages."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            '--gradle-args',
            nargs='*', metavar='*', type=str.lstrip,
            help='Pass arguments to Gradle projects. '
            'Arguments matching other options must be prefixed by a space,\n'
            'e.g. --gradle-args " --help"')
        parser.add_argument(
            '--gradle-task',
            help='Run a specific task instead of the default task')

    async def build(
        self, *, additional_hooks=None, skip_hook_creation=False
    ):  # noqa: D102
        pkg = self.context.pkg
        args = self.context.args

        logger.info(
            "Building Gradle package in '{args.path}'".format_map(locals()))

        try:
            env = await get_command_environment(
                'build', args.build_base, self.context.dependencies)
        except RuntimeError as e:
            logger.error(str(e))
            return 1

        rc = await self._build(args, env)
        if rc and rc.returncode:
            return rc.returncode

        if not skip_hook_creation:
            create_environment_scripts(
                pkg, args, additional_hooks=additional_hooks)

    async def _build(self, args, env):
        self.progress('build')

        # invoke build step
        if GRADLE_EXECUTABLE is None:
            raise RuntimeError("Could not find 'gradle' executable")
        cmd = [GRADLE_EXECUTABLE]
        if args.gradle_task:
            cmd += [args.gradle_task]
        else:
            cmd += ['assemble']
        return await check_call(
            self.context, cmd, cwd=args.build_base, env=env)
