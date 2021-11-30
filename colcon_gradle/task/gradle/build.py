# Copyright 2018 Esteve Fernandez
# Licensed under the Apache License, Version 2.0

from distutils import dir_util
import glob
import os
from pathlib import Path
import shutil

from colcon_core.environment import create_environment_scripts
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.shell import create_environment_hook
from colcon_core.shell import get_command_environment
from colcon_core.task import run
from colcon_core.task import TaskExtensionPoint
from colcon_gradle.task.gradle import get_wrapper_executable
from colcon_gradle.task.gradle import GRADLE_EXECUTABLE
from colcon_gradle.task.gradle import has_wrapper_executable


logger = colcon_logger.getChild(__name__)


class GradleBuildTask(TaskExtensionPoint):
    """Build gradle packages."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def _build_file_tree(self, start_path):
        out_dirnames = set()
        out_filenames = set()
        for dirname, dirnames, filenames in os.walk(start_path):
            for subdirname in dirnames:
                out_dirnames.add(
                    os.path.relpath(
                        os.path.join(dirname, subdirname), start=start_path))

            for filename in filenames:
                out_filenames.add(
                    os.path.relpath(
                        os.path.join(dirname, filename), start=start_path))
        return (out_dirnames, out_filenames)

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

    async def build(  # noqa: D102
        self, *, additional_hooks=None, skip_hook_creation=False
    ):
        pkg = self.context.pkg
        args = self.context.args

        logger.info(
            "Building Gradle package in '{args.path}'".format_map(locals()))

        if additional_hooks is None:
            additional_hooks = []

        # add jars and classes to CLASSPATH with wildcards
        # https://docs.oracle.com/javase/8/docs/technotes/tools/windows/classpath.html#A1100762
        additional_hooks += create_environment_hook(
            'classpath_jars', Path(args.install_base), pkg.name,
            'CLASSPATH', os.path.join('share', pkg.name, 'java', '*'),
            mode='prepend')
        additional_hooks += create_environment_hook(
            'classpath_classes', Path(args.install_base), pkg.name,
            'CLASSPATH', os.path.join('share', pkg.name, 'java'),
            mode='prepend')

        try:
            env = await get_command_environment(
                'build', args.build_base, self.context.dependencies)
        except RuntimeError as e:
            logger.error(str(e))
            return 1

        rc = await self._build(args, env)
        if rc and rc.returncode:
            return rc.returncode

        rc = await self._install(args, env)
        if rc and rc.returncode:
            return rc.returncode

        if not skip_hook_creation:
            create_environment_scripts(
                pkg, args, additional_hooks=additional_hooks)

    async def _build(self, args, env):
        self.progress('build')

        # remove anything on the destination tree but not in the source tree
        src_package_src_dir = os.path.join(args.path, 'src')
        dst_package_src_dir = os.path.join(args.build_base, 'src')

        src_dirnames, src_filenames = self._build_file_tree(
            src_package_src_dir)
        dst_dirnames, dst_filenames = self._build_file_tree(
            dst_package_src_dir)

        prune_dirnames = dst_dirnames - src_dirnames
        prune_filenames = dst_filenames - src_filenames

        for prune_filename in prune_filenames:
            os.remove(os.path.join(dst_package_src_dir, prune_filename))
        for prune_dirname in prune_dirnames:
            if os.path.exists(prune_dirname):
                shutil.rmtree(os.path.join(dst_package_src_dir, prune_dirname))

        # copy files from the source directory to the build one to avoid
        # polluting the latter during the build process
        dir_util.copy_tree(args.path, args.build_base, update=1)

        # Gradle Executable
        if has_wrapper_executable(args):
            cmd = [str(get_wrapper_executable(args).absolute())]
        elif GRADLE_EXECUTABLE is not None:
            cmd = [GRADLE_EXECUTABLE]
        else:
            raise RuntimeError(
                "Could not find 'gradle' or 'wrapper' executable")

        # Gradle Task (by default 'assemble')
        if args.gradle_task:
            cmd += [args.gradle_task]
        else:
            cmd += ['assemble']

        # Gradle Arguments
        cmd += (args.gradle_args or [])
        cmd += ['--stacktrace']

        # Add install_base to environment in GRADLE_INSTALL_PREFIX
        env['GRADLE_INSTALL_PREFIX'] = args.install_base

        # invoke build step
        return await run(
            self.context, cmd, cwd=args.build_base, env=env)

    async def _install(self, args, env):
        self.progress('install')
        pkg = self.context.pkg

        # remove anything on the destination tree but not in the build tree
        bld_package_jar_dir = os.path.join(args.build_base, 'build', 'libs')
        dst_package_jar_dir = os.path.join(
            args.install_base, 'share', pkg.name, 'java')
        os.makedirs(dst_package_jar_dir, exist_ok=True)

        bld_dirnames, bld_filenames = self._build_file_tree(
            bld_package_jar_dir)
        dst_dirnames, dst_filenames = self._build_file_tree(
            dst_package_jar_dir)

        prune_dirnames = dst_dirnames - bld_dirnames
        prune_filenames = dst_filenames - bld_filenames

        for prune_filename in prune_filenames:
            os.remove(os.path.join(dst_package_jar_dir, prune_filename))
        for prune_dirname in prune_dirnames:
            if os.path.exists(prune_dirname):
                shutil.rmtree(
                    os.path.join(dst_package_jar_dir, prune_dirname))

        for jar in glob.glob(os.path.join(bld_package_jar_dir, '*.jar')):
            jar_filename = os.path.basename(jar)
            shutil.copy2(jar, os.path.join(dst_package_jar_dir, jar_filename))
