# Copyright 2018 Esteve Fernandez
# Licensed under the Apache License, Version 2.0

import os
from pathlib import Path
import shutil
import subprocess

from colcon_core.environment_variable import EnvironmentVariable
from colcon_core.subprocess import check_output

"""Environment variable to override the Gradle executable"""
GRADLE_COMMAND_ENVIRONMENT_VARIABLE = EnvironmentVariable(
    'GRADLE_COMMAND', 'The full path to the Gradle executable')
    
"""Environment variable to override the Gradle executable"""
GRADLE_HOME_ENVIRONMENT_VARIABLE = EnvironmentVariable(
    'GRADLE_HOME', 'The full path to the Gradle home')

"""Check OS"""
IS_WINDOWS = os.name == 'nt'

"""Check OS"""
IS_WINDOWS = os.name == 'nt'

def which_executable(environment_variable, executable_name):
    """
    Determine the path of an executable.

    An environment variable can be used to override the location instead of
    relying on searching the PATH.

    :param str environment_variable: The name of the environment variable
    :param str executable_name: The name of the executable
    :rtype: str
    """
    cmd = None
    env_cmd = os.getenv(environment_variable)
    env_home = os.getenv(GRADLE_HOME_ENVIRONMENT_VARIABLE.name)
    
    # Case of GRADLE_COMMAND (colcon)
    if env_cmd is not None and Path(env_cmd).is_file():
        cmd = env_cmd

    # Case of GRADLE_HOME (official)
    if cmd is None and env_home is not None:
        gradle_path = Path(env_home) / 'bin' / executable_name
        if gradle_path.is_file():
            cmd = gradle_path

    # fallback (from PATH)
    if cmd is None:
        cmd = shutil.which(executable_name)

    return cmd

GRADLE_EXECUTABLE = which_executable(
    GRADLE_COMMAND_ENVIRONMENT_VARIABLE.name, 'gradle')


async def has_task(path, task):
    """
    Check if the Gradle project has a specific task.

    :param str path: The path of the directory containing the build.gradle file
    :param str target: The name of the target
    :rtype: bool
    """
    return task in await get_gradle_tasks(path)


async def get_gradle_tasks(path):
    """
    Get all targets from a `build.gradle`.

    :param str path: The path of the directory contain the build.gradle file
    :returns: The target names
    :rtype: list
    """
    output = await check_output([
        GRADLE_EXECUTABLE, 'tasks'], cwd=path)
    lines = output.decode().splitlines()
    separator = ' - '
    return [l.split(separator)[0] for l in lines if separator in l]

def has_local_executable(args):
    """
    Check if Gradle wrapper executable is available on project.
    
    :param Arguments args: 
    :returns: True if exist
    :rtype: bool
    """
    gradle_path = get_local_executable(args)
    return gradle_path.is_file()

def get_local_executable(args):
    """
    Get Gradle wrapper executable.
    
    :param Arguments args: Argument 
    :returns: The path of Gradle Wrapper executable
    :rtype: Path
    """
    gradle_script = 'gradlew.bat' if IS_WINDOWS else 'gradlew'
    gradle_path = Path(args.path) / gradle_script
    return gradle_path
