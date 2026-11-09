# Build Native Applications

Builds use PyInstaller to bundle the Python interpreter, Tcl/Tk, Matplotlib,
NumPy, and required libraries. Recipients do not install Python, pip, or a browser.
Only the person producing builds needs Python and the build tools.

## Supported Outputs

| Platform | Output | Distribution archive |
| --- | --- | --- |
| Windows 10/11, x86_64 | `SensorTelemetry.exe`, optionally with a runtime folder | `.zip` |
| macOS, Apple Silicon (arm64) | `SensorTelemetry.app` | `.tar.gz` |
| macOS, Intel (x86_64) | `SensorTelemetry.app` | `.tar.gz` |
| Linux, x86_64 or arm64 | `SensorTelemetry` executable, with an optional runtime folder and `launch.sh` | `.tar.gz` |

Each artifact targets one OS and CPU architecture. **PyInstaller is not a
cross-compiler.** Build Windows on Windows, macOS on macOS, and Linux on Linux.
The build script refuses a mismatched `--target` instead of producing a mislabeled file.
Use the GitHub Actions workflow below to build all five targets from one repository.

## Easiest Local Build

Extract the entire build kit first, then open its `python_app` folder.
Use Python 3.10 or later; CI uses Python 3.12. Build launchers create a local
`.build-venv` and install the Python dependencies. Internet is needed for that
initial installation, not for running the completed application.

| OS | Launcher |
| --- | --- |
| Windows | Double-click `build-windows.bat` |
| macOS | Run `sh build-macos.command`, or `chmod +x build-macos.command` and double-click it |
| Linux | Run `sh build-linux.sh` |

The launchers use `build_options.json`. A downloaded kit records the selected
layout for the selected platform there, and includes your current calibration
profile if requested. Other platforms retain the recommended folder layout.
The desktop application itself also offers **File > Build Executable Application**
when launched from this source project in a Python environment with build tools installed.
That dialog includes current calibration edits, streams real build logs, and opens
the output folder after a successful build. Building is disabled inside a frozen
application: its embedded runtime is not a development environment.

## Manual Build

Run these commands in `python_app`. Use `python3` instead of `python` where appropriate.

```sh
python -m venv .build-venv
```

On Windows, use `.build-venv\Scripts\python.exe` for `python` in the next commands.
On macOS/Linux, use `.build-venv/bin/python`.

```sh
python -m pip install -r requirements-build.txt
python build_app.py --mode onedir
```

The script detects the OS and interpreter architecture automatically. Optional
examples, each run on its matching operating system:

```sh
python build_app.py --target windows --mode onefile
python build_app.py --target macos --mode onedir
python build_app.py --target linux --mode onefile
python build_app.py --calibrations my_calibrations.json --version 1.2.0
python build_app.py --entry sensor_analyzer_desktop.py
python build_app.py --output /path/to/builds
python build_app.py --dry-run
```

`--mode onedir` is recommended: startup is faster and runtime files are easy to inspect.
Distribute the **whole folder**, not just its executable. `--mode onefile` is
available for Windows and Linux; it extracts runtime files into a temporary
directory at startup. Linux one-file builds need an executable temporary filesystem.
macOS always uses an onedir `.app` bundle, following PyInstaller's guidance.
No administrator privileges are needed to launch the app.

## Build Prerequisites

- Windows: Install 64-bit Python from python.org with Tcl/Tk support. Match the
  interpreter architecture to the desired output. The included CI job produces x86_64.
- macOS: Use a Python distribution with working Tcl/Tk. Verify with `python3 -m tkinter`.
  Native arm64 and x86_64 builds are separate; an ARM build is not an Intel build.
  The included CI uses macOS 15. Older macOS releases are not guaranteed to work;
  build on the oldest macOS you intend to support and test your dependencies there.
- Debian/Ubuntu: `sudo apt-get install python3-tk python3-venv libpython3-dev binutils`.
- Fedora/RHEL: Install the matching `python3-tkinter`, `python3-pip`, and `binutils`
  packages for your Python version.
- Arch: Install `python`, `python-pip`, `tk`, and `binutils`.
- Headless Linux CI also needs `xvfb` and `xauth`. Run the builder with
  `xvfb-run -a python build_app.py` so its GUI smoke test can open a display.

Tkinter is an optional operating-system component on some Python distributions;
it is not installed by pip. PyInstaller's hooks collect Tcl/Tk from the build environment.
The generated spec explicitly includes TkAgg plus the PNG/PDF/SVG rendering backends.
Minimal system-Python installations may also need the matching `libpython3.x`
shared library; installing `libpython3-dev` on Debian/Ubuntu supplies it.

## Find And Share Builds

Default output:

```text
python_app/dist/native/<platform>-<architecture>-<mode>/
  app/         Native application and bundled runtime
  releases/    Shareable archive, build metadata JSON, SHA-256 checksums
```

Only share the archive from `releases/`, after checking it on a clean target machine.
It contains the complete runtime, opening instructions, and build metadata.
`.tar.gz` is used for Unix platforms to preserve executable permissions and
macOS `.app` framework symlinks. End users extract the archive, then open the
`.exe` on Windows, move/open the `.app` on macOS, or launch `launch.sh` on Linux.
Linux file managers differ: a terminal `./launch.sh` is a fallback if double-click
does not execute scripts. Match both the OS and the CPU architecture.

Each build launches the **packaged executable** with `--smoke-test`, creates a
real Tk window, draws the chart, checks calibration math, and exports PNG, PDF,
and SVG in memory. The process must finish successfully before a release archive
is created. `--skip-smoke-test` exists for unusual build environments, but marks
the metadata as untested and should not be used for public releases.
Checksums verify integrity, not publisher identity.

## Build All Platforms With GitHub Actions

Commit the kit's `python_app/` directory and `.github/workflows/build-desktop.yml`
at your repository root. In GitHub, open **Actions > Build Desktop Applications >
Run workflow**. Choose an onedir or onefile layout for Windows/Linux. macOS stays onedir.
The workflow also runs for tags such as `sensor-v1.2.0` and relevant pull requests.
Set `version` in `python_app/build_options.json` before tagging; the tag is a
trigger, not an automatic version override.

The five jobs use Windows x86_64, macOS Intel, macOS Apple Silicon, Linux x86_64,
and Linux arm64 runners. Each installs build tools, runs the unit tests, builds,
smoke-tests the frozen executable, and uploads its release archive, metadata,
and checksums as an Actions artifact. Download them from the completed workflow
run. Artifacts expire after 30 days; publish reviewed builds to a release yourself.
The workflow needs no signing secrets and does not claim to produce publisher-signed apps.
Availability, quotas, and billing for hosted runners depend on your GitHub plan.

## Linux Distribution Compatibility

Linux does not have one executable that is guaranteed to run on every distribution.
PyInstaller does **not** bundle glibc, the kernel, or the display server. Build on
the oldest glibc-based distribution you support and test on each target distro.
The included Linux CI uses Ubuntu 22.04 (glibc 2.35) as its baseline; those
artifacts are not promised to work with an older glibc. Local builds record their
actual libc and OS in `build-info.json`. Newer glibc alone does not prove full
compatibility; system-library and graphics/display differences still need testing.
Alpine/musl is not compatible with these glibc builds; build separately on musl
and verify there if needed. A desktop session with compatible X11/XWayland
libraries is required. This kit does not generate AppImage, Flatpak, DEB, or RPM
installers. The portable archive works without a package-manager installation
on compatible target systems.

## Signing And Public Distribution

The default output is an unsigned Windows executable or an ad-hoc-signed macOS
app, **not** a trusted publisher-signed release. SmartScreen/Gatekeeper warnings
can occur. Do not ask recipients to disable OS security protections.

For Windows, sign the built executable with a trusted Authenticode certificate
using Microsoft's SignTool, then recreate the ZIP and SHA-256 checksum. Protect
the certificate and use an approved timestamp server from your certificate provider.

For macOS, build using a Developer ID Application identity installed in the keychain:

```sh
python build_app.py --target macos --mode onedir \
  --codesign-identity "Developer ID Application: Your Organization (TEAMID)"
```

For public distribution, also notarize and staple the app. Example, substituting
your actual build path and an existing notarytool keychain profile:

```sh
ditto -c -k --sequesterRsrc --keepParent /path/to/SensorTelemetry.app SensorTelemetry-notarize.zip
xcrun notarytool submit SensorTelemetry-notarize.zip --keychain-profile YOUR_PROFILE --wait
xcrun stapler staple /path/to/SensorTelemetry.app
codesign --verify --deep --strict /path/to/SensorTelemetry.app
spctl --assess --type execute /path/to/SensorTelemetry.app
```

Recreate the distribution archive and checksums after stapling. Certificate setup,
notarization credentials, Windows signing, and App Store submission are not
automated by the supplied workflow. Test on clean Windows/macOS machines before
sharing outside your development environment.

## Tests

```sh
python -B -m unittest discover -s . -v
python build_app.py --dry-run
```

Official references: [PyInstaller usage](https://pyinstaller.org/en/stable/usage.html),
[platform compatibility and code signing](https://pyinstaller.org/en/stable/feature-notes.html),
and [Matplotlib hook configuration](https://pyinstaller.org/en/stable/hooks-config.html#matplotlib-hooks).