# Copyright 2018 Esteve Fernandez
# Licensed under the Apache License, Version 2.0

import os

from colcon_core.event.test import TestFailure
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.shell import get_command_environment
from colcon_core.subprocess import check_output
from colcon_core.task import check_call
from colcon_core.task import TaskExtensionPoint
from colcon_gradle.task.gradle import has_local_executable
from colcon_gradle.task.gradle import get_local_executable


logger = colcon_logger.getChild(__name__)


class GradleTestTask(TaskExtensionPoint):
    """Test Gradle packages."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            '--gradletest-args',
            nargs='*', metavar='*', type=str.lstrip,
            help='Pass arguments to Gradle projects. '
            'Arguments matching other options must be prefixed by a space,\n'
            'e.g. --gradletest-args " --help"')
        parser.add_argument(
            '--gradle-task',
            help='Run a specific task instead of the default task')

    async def test(self, *, additional_hooks=None):  # noqa: D102
        pkg = self.context.pkg
        args = self.context.args
        
        logger.info(
            "Testing Gradle package in '{args.path}'".format_map(locals()))

        try:
            env = await get_command_environment(
                'test', args.build_base, self.context.dependencies)
        except RuntimeError as e:
            logger.error(str(e))
            return 1
        
        rc = await self._test(args, env)
        if rc and rc.returncode:
            return rc.returncode

    async def _test(self, args, env):
        self.progress('test')

        # Gradle Executable
        if has_local_executable(args):
            cmd = [str(get_local_executable(args).absolute())]
        elif GRADLE_EXECUTABLE is not None:
            cmd = [GRADLE_EXECUTABLE]
        else:
            msg = "Could not find 'gradle' or 'wrapper' executable"
            logger.error(msg)
            raise RuntimeError(msg)

        # Gradle Task (by default 'test')
        if args.gradle_task:
            cmd += [args.gradle_task]
        else:
            cmd += ['test']

        # Gradle Arguments
        if args.gradletest_args:
            cmd += args.gradletest_args

        # invoke build step
        return await check_call(
            self.context, cmd, cwd=args.build_base, env=env)

