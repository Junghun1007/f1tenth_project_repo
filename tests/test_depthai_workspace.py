"""Validate SDK selection and runtime loading without ROS or an OAK device."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


REPOSITORY = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which("cmake"), "CMake unavailable")
class DepthaiWorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="depthai workspace ")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.workspace = self.root / "f1tenth_project_repo"
        shutil.copytree(REPOSITORY / "cmake", self.workspace / "cmake")
        self.sdk = self.root / "depthai-core" / "install"
        self.library_dir = self.sdk / "lib"
        self.library_dir.mkdir(parents=True)
        self.library = self.library_dir / (
            "libdepthai-core.dylib" if sys.platform == "darwin" else "libdepthai-core.so"
        )
        self.library.touch()
        self.config = self.sdk / "lib/cmake/depthai"
        self.config.mkdir(parents=True)
        self.write_sdk_config()
        (self.config / "depthaiConfigVersion.cmake").write_text(
            'set(PACKAGE_VERSION "3.6.1")\n'
            'if(PACKAGE_FIND_VERSION VERSION_GREATER PACKAGE_VERSION)\n'
            '  set(PACKAGE_VERSION_COMPATIBLE FALSE)\n'
            'else()\n'
            '  set(PACKAGE_VERSION_COMPATIBLE TRUE)\n'
            'endif()\n'
        )
        self.source = self.workspace / "src/probe"
        self.source.mkdir(parents=True)
        self.build = self.root / "probe build"
        self.write_project()

    def write_sdk_config(self, kind="SHARED"):
        (self.config / "depthaiConfig.cmake").write_text(
            f'add_library(depthai::core {kind} IMPORTED)\n'
            'set_target_properties(depthai::core PROPERTIES\n'
            '  IMPORTED_CONFIGURATIONS RELEASE\n'
            f'  IMPORTED_LOCATION_RELEASE "{self.library}")\n'
        )

    def write_project(self, *, executable=False):
        (self.source / "CMakeLists.txt").write_text(
            'cmake_minimum_required(VERSION 3.16)\n'
            f'project(probe {"CXX" if executable else "NONE"})\n'
            'function(ament_environment_hooks template)\n'
            '  configure_file("${template}" "${CMAKE_CURRENT_BINARY_DIR}/sdk.sh" @ONLY)\n'
            'endfunction()\n'
            'include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/DepthaiWorkspace.cmake")\n'
            'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/selection.txt"\n'
            '  "${depthai_DIR}\\n${CMAKE_INSTALL_RPATH}\\n")\n'
            + ('add_executable(probe main.cpp)\n'
               'target_link_libraries(probe PRIVATE depthai::core)\n'
               'install(TARGETS probe DESTINATION bin)\n' if executable else '')
        )

    def configure(self, *arguments, success=True):
        result = subprocess.run(
            ["cmake", "-S", str(self.source), "-B", str(self.build),
             "-DCMAKE_BUILD_TYPE=Release", *arguments],
            text=True, capture_output=True,
        )
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def test_ignores_stale_cache_and_other_prefixes(self):
        stale = self.root / "previous SDK"
        stale.mkdir()
        (stale / "depthaiConfig.cmake").write_text('message(FATAL_ERROR "WRONG SDK")\n')
        self.configure(f"-Ddepthai_DIR={stale}", f"-DCMAKE_PREFIX_PATH={stale}")
        self.assertEqual(
            (self.build / "selection.txt").read_text().splitlines(),
            [str(self.config), str(self.library_dir)],
        )
        self.assertEqual(
            (self.build / "depthai_runtime_dir.txt").read_text(),
            str(self.library_dir) + "\n",
        )
        self.assertIn(
            f"depthai_DIR:PATH={self.config}",
            (self.build / "CMakeCache.txt").read_text(),
        )

    def test_missing_sibling_sdk_fails_instead_of_using_another_install(self):
        other_sdk = self.root / "other SDK"
        self.sdk.rename(other_sdk)
        result = self.configure(
            f"-Ddepthai_DIR={other_sdk / 'lib/cmake/depthai'}",
            f"-DCMAKE_PREFIX_PATH={other_sdk}", success=False,
        )
        self.assertIn("scripts/install_depthai.sh", result.stderr)
        self.assertIn("workspace DepthAI C++ SDK is missing", result.stderr)

    def test_missing_library_fails_with_setup_command(self):
        self.library.unlink()
        result = self.configure(success=False)
        self.assertIn("scripts/install_depthai.sh", result.stderr)

    def test_static_sdk_is_rejected(self):
        self.write_sdk_config(kind="STATIC")
        result = self.configure(success=False)
        self.assertIn("BUILD_SHARED_LIBS=ON", result.stderr)

    def test_sdk_export_cannot_point_to_an_external_library(self):
        self.library = self.root / "external-depthai.so"
        self.library.touch()
        self.write_sdk_config()
        result = self.configure(success=False)
        self.assertIn("outside the sibling depthai-core", result.stderr)

    def test_installer_uses_sibling_checkout_from_another_working_directory(self):
        shutil.copytree(REPOSITORY / "scripts", self.workspace / "scripts")
        sdk_source = self.sdk.parent
        (sdk_source / "CMakeLists.txt").write_text(
            'cmake_minimum_required(VERSION 3.16)\nproject(sdk NONE)\n'
            'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/sdk-settings.txt"\n'
            '  "${CMAKE_INSTALL_PREFIX}\\n${CMAKE_BUILD_TYPE}\\n"\n'
            '  "${BUILD_SHARED_LIBS}\\n${DEPTHAI_OPENCV_SUPPORT}\\n")\n'
            'install(FILES "${CMAKE_CURRENT_BINARY_DIR}/sdk-settings.txt" DESTINATION .)\n'
        )
        result = subprocess.run(
            ["bash", str(self.workspace / "scripts/install_depthai.sh")],
            cwd=self.root, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(
            (self.sdk / "sdk-settings.txt").read_text().splitlines(),
            [str(self.sdk), "Release", "ON", "ON"],
        )

    def test_hook_moves_sdk_ahead_of_conflicting_paths_on_every_source(self):
        self.configure()
        variable = "DYLD_LIBRARY_PATH" if sys.platform == "darwin" else "LD_LIBRARY_PATH"
        previous = str(self.root / "other SDK/lib")
        environment = dict(os.environ)
        environment[variable] = f"{previous}:{self.library_dir}:{self.library_dir}"
        environment["CMAKE_PREFIX_PATH"] = f"{previous}:{self.sdk}:{self.sdk}"
        script = (
            # macOS strips DYLD_* before starting /bin/bash (SIP), so set
            # the inherited value inside the shell before exercising the hook.
            f'export {variable}="$2"\n'
            '. "$1"\n. "$1"\n'
            f'printf "%s\\n" "${{{variable}}}" "$CMAKE_PREFIX_PATH"\n'
        )
        result = subprocess.run(
            ["bash", "-c", script, "hook-test", str(self.build / "sdk.sh"),
             environment[variable]],
            env=environment, text=True, capture_output=True, check=True,
        )
        self.assertEqual(result.stdout.splitlines(), [
            f"{self.library_dir}:{previous}", f"{self.sdk}:{previous}",
        ])

    @unittest.skipUnless(shutil.which("c++"), "C++ compiler unavailable")
    def test_real_executable_uses_workspace_sdk_with_conflicting_runtime_library(self):
        # Two actual shared libraries have the same SONAME/install name but
        # incompatible behavior, reproducing a wrong runtime SDK selection.
        self.library.unlink()
        fixture = self.root / "fixture"
        fixture.mkdir()
        (fixture / "sdk.cpp").write_text('extern "C" int sdk_generation() { return SDK_VERSION; }\n')
        conflict = self.root / "old SDK/lib"
        (fixture / "CMakeLists.txt").write_text(
            'cmake_minimum_required(VERSION 3.16)\nproject(fixture CXX)\n'
            'foreach(name current old)\n'
            '  add_library(${name} SHARED sdk.cpp)\n'
            '  set_target_properties(${name} PROPERTIES OUTPUT_NAME depthai-core\n'
            '    INSTALL_NAME_DIR "@rpath" BUILD_WITH_INSTALL_NAME_DIR ON)\n'
            'endforeach()\n'
            f'set_target_properties(current PROPERTIES LIBRARY_OUTPUT_DIRECTORY "{self.library_dir}")\n'
            f'set_target_properties(old PROPERTIES LIBRARY_OUTPUT_DIRECTORY "{conflict}")\n'
            'target_compile_definitions(current PRIVATE SDK_VERSION=3)\n'
            'target_compile_definitions(old PRIVATE SDK_VERSION=2)\n'
        )
        for command in (
            ["cmake", "-S", str(fixture), "-B", str(fixture / "build")],
            ["cmake", "--build", str(fixture / "build")],
        ):
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.write_project(executable=True)
        (self.source / "main.cpp").write_text(
            'extern "C" int sdk_generation();\n'
            'int main() { return sdk_generation() == 3 ? 0 : 9; }\n'
        )
        install = self.root / "probe install"
        self.configure(f"-DCMAKE_INSTALL_PREFIX={install}")
        for command in (
            ["cmake", "--build", str(self.build)],
            ["cmake", "--install", str(self.build)],
        ):
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        executable = install / "bin/probe"
        self.assertEqual(subprocess.run([str(executable)]).returncode, 0)
        variable = "DYLD_LIBRARY_PATH" if sys.platform == "darwin" else "LD_LIBRARY_PATH"
        environment = dict(os.environ, **{variable: str(conflict)})
        self.assertEqual(subprocess.run([str(executable)], env=environment).returncode, 9)
        environment[variable] += os.pathsep + str(self.library_dir)
        result = subprocess.run(
            ["bash", "-c", f'export {variable}="$3"; . "$1"; exec "$2"',
             "runtime-test", str(self.build / "sdk.sh"), str(executable),
             environment[variable]],
            env=environment, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
