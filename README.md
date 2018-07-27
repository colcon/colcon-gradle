colcon-gradle
=============

[![Travis CI](https://travis-ci.org/colcon/colcon-gradle.svg?branch=master)](https://travis-ci.org/colcon/colcon-gradle)
[![AppVeyor](https://ci.appveyor.com/api/projects/status/github/colcon/colcon-gradle?svg=true&branch=master)](https://ci.appveyor.com/project/esteve/colcon-gradle)

An extension for [colcon-core](https://github.com/colcon/colcon-core) to support [Gradle](https://gradle.org) projects.

## Features

For all packages with `build.gradle` files:

- `colcon build` will call `gradle assemble`
- `colcon test` will call `gradle test`

Gradle wrappers will be used if present.

## Try it out

Follow the instructions at https://colcon.readthedocs.io/en/latest/developer/bootstrap.html, except in "Fetch the sources" add the following to `colcon.repos`:

```yaml
  colcon-gradle:
    type: git
    url: https://github.com/colcon/colcon-gradle.git
    version: master
```

After that, run the `local_setup` file, build any colcon workspace with Gradle projects in it, and report any issues!
