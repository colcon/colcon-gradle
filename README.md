# colcon-gradle

[![Run tests](https://github.com/colcon/colcon-gradle/actions/workflows/ci.yaml/badge.svg)](https://github.com/colcon/colcon-gradle/actions/workflows/ci.yaml)

An extension for [colcon-core](https://github.com/colcon/colcon-core) to support [Gradle](https://gradle.org) projects.

## Features

For all packages with `build.gradle` files:

- `colcon build` will call `gradle assemble`
- `colcon test` will call `gradle test`

Gradle wrappers will be used if present.

## Try it out

### Using pip

```
pip install -U colcon-gradle
```

### From source

Follow the instructions at https://colcon.readthedocs.io/en/released/developer/bootstrap.html, except in "Fetch the sources" add the following to `colcon.repos`:

```yaml
  colcon-gradle:
    type: git
    url: https://github.com/colcon/colcon-gradle.git
    version: main
```

After that, run the `local_setup` file, build any colcon workspace with Gradle projects in it, and report any issues!
