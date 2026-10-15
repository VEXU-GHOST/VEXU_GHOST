# ghost_comms

Two standalone PROS projects used to bench-test **VEXlink** radio communication
between two V5 brains, plus USB-serial IMU streaming from the V5 to the Jetson.

| Project   | VEXlink role        | Sends                                              | Receives              |
| --------- | ------------------- | -------------------------------------------------- | --------------------- |
| `manager` | `pros::E_LINK_TX`   | pose over radio (5 Hz), IMU over USB serial (50 Hz) | —                     |
| `worker`  | `pros::E_LINK_RX`   | —                                                   | pose over radio       |

## Layout

```
ghost_comms/
├── pros_template/   PROS 4.2.2 + liblvgl 9.2.0 template files, vendored once
├── common/          vexlink_protocol.hpp — the wire format, shared by both ends
├── manager/         radio TX + IMU serial
└── worker/          radio RX + LCD readout
```

Each project keeps only the files PROS considers yours — `Makefile`,
`project.pros`, `src/`, `include/main.h`. Everything else is a symlink into
`pros_template/` or `common/`, and is listed in the repo's root `.gitignore`
so the template is stored once rather than once per project. This follows the
same pattern `scripts/update_symlinks.sh` uses for `ghost_pros`.

## Hardware setup

Ports are set at the top of each `src/main.cpp`:

- `manager`: radio on Smart Port 11, IMU on Smart Port 1
- `worker`: radio on Smart Port 10

`LINK_ID` must be **byte-identical on both brains** (`"my_vexlink"` by default)
or the radios will never pair. Both brains must be running their program for the
link to come up; the worker's LCD shows `NOT LINKED` until it does.

## Build and upload

```sh
bash scripts/pros_build.sh 02_V5/ghost_comms/manager
bash scripts/pros_upload.sh 02_V5/ghost_comms/worker
```

Both scripts default to `02_V5/ghost_pros` when given no argument, and skip the
symlink/`robot_config.hpp` generation steps for anything else.

## Maintenance

**`pros c apply` replaces symlinks with real files.** After upgrading a template,
re-point them:

```sh
cd $VEXU_HOME/02_V5/ghost_comms && for p in manager worker; do
  (cd $p && ln -sfn ../pros_template/common.mk common.mk \
         && ln -sfn ../pros_template/firmware firmware \
         && cd include \
         && ln -sfn ../../pros_template/include/pros pros \
         && ln -sfn ../../pros_template/include/liblvgl liblvgl \
         && ln -sfn ../../pros_template/include/api.h api.h \
         && ln -sfn ../../common/vexlink_protocol.hpp vexlink_protocol.hpp)
done
```

The `templates.*.location` field in each `project.pros` is machine-local — it
records wherever the template cache sat on whoever last ran `pros c apply`, so
it will not match your machine. `pros make` and `pros upload` ignore it;
only `pros c upgrade` reads it, and running `pros c apply kernel@4.2.2` locally
rewrites it. Don't bother "fixing" it in git.

## Conventions

C++ here follows the repo's `uncrustify.cfg` (2-space indent). Run it before
opening a PR, as `.github/pull_request_template.md` requires:

```sh
uncrustify -c uncrustify.cfg --no-backup 02_V5/ghost_comms/*/src/*.cpp 02_V5/ghost_comms/common/*.hpp
```
