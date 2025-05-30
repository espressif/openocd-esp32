import copy
import glob
import os
import os.path
import re
import shutil
import tempfile


def action_extensions(base_actions, project_path=os.getcwd()):
    """ Describes extensions for unit tests. This function expects that actions "all" and "reconfigure" """

    PROJECT_NAME = "gen_ut_app"

    # List of unit-test-app configurations.
    # Each file in configs/ directory defines a configuration. The format is the
    # same as sdkconfig file. Configuration is applied on top of sdkconfig.defaults
    # file from the project directory
    CONFIG_NAMES = os.listdir(os.path.join(project_path, "configs"))

    # Build (intermediate) and output (artifact) directories
    BUILDS_DIR = os.path.join(project_path, "builds")
    BINARIES_DIR = os.path.join(project_path, "output")

    def parse_file_to_dict(path, regex):
        """
        Parse the config file at 'path'

        Returns a dict of name:value.
        """
        compiled_regex = re.compile(regex)
        result = {}
        with open(path) as f:
            for line in f:
                m = compiled_regex.match(line)
                if m:
                    result[m.group(1)] = m.group(2)
        return result

    def parse_config(path):
        """
        Expected format with default regex is "key=value"
        """

        return parse_file_to_dict(path, r"^([^=]+)=(.+)$")

    def ut_apply_config(ut_apply_config_name, ctx, args):
        config_name = re.match(r"ut-apply-config-(.*)", ut_apply_config_name).group(1)
        # Make sure that define_cache_entry is list
        args.define_cache_entry = list(args.define_cache_entry)
        new_cache_values = {}
        sdkconfig_set = list(filter(lambda s: "SDKCONFIG=" in s, args.define_cache_entry))
        sdkconfig_path = os.path.join(args.project_dir, "sdkconfig")

        if sdkconfig_set:
            sdkconfig_path = sdkconfig_set[-1].split("=")[1]
            sdkconfig_path = os.path.abspath(sdkconfig_path)

        try:
            os.remove(sdkconfig_path)
        except OSError:
            pass

        if config_name in CONFIG_NAMES:
            # Parse the sdkconfig for components to be included/excluded and tests to be run
            config_path = os.path.join(project_path, "configs", config_name)
            config = parse_config(config_path)

            with tempfile.NamedTemporaryFile() as sdkconfig_temp:
                # Use values from the combined defaults, chip config defaults and the values from
                # config folder to build config
                sdkconfig_default = os.path.join(project_path, "sdkconfig.defaults")
                with open(sdkconfig_default, "rb") as sdkconfig_default_file:
                    sdkconfig_temp.write(sdkconfig_default_file.read())

                sdkconfig_chip_default = os.path.join(project_path, "sdkconfig.defaults." + os.getenv('IDF_TARGET'))
                if os.path.exists(sdkconfig_chip_default):
                    with open(sdkconfig_chip_default, "rb") as sdkconfig_chip_default_file:
                        sdkconfig_temp.write(sdkconfig_chip_default_file.read())

                sdkconfig_config = os.path.join(project_path, "configs", config_name)
                with open(sdkconfig_config, "rb") as sdkconfig_config_file:
                    sdkconfig_temp.write(b"\n")
                    sdkconfig_temp.write(sdkconfig_config_file.read())

                sdkconfig_temp.flush()
                new_cache_values["SDKCONFIG_DEFAULTS"] = sdkconfig_temp.name

                args.define_cache_entry.extend(["%s=%s" % (k, v) for k, v in new_cache_values.items()])

                reconfigure = base_actions["actions"]["reconfigure"]["callback"]
                reconfigure(None, ctx, args)

    # This target builds the configuration. It does not currently track dependencies,
    # but is good enough for CI builds if used together with clean-all-configs.
    # For local builds, use 'apply-config-NAME' target and then use normal 'all'
    # and 'flash' targets.
    def ut_build(ut_build_name, ctx, args):
        # Create a copy of the passed arguments to prevent arg modifications to accrue if
        # all configs are being built
        build_args = copy.copy(args)

        config_name = re.match(r"ut-build-(.*)", ut_build_name).group(1)

        if config_name in CONFIG_NAMES:
            build_args.build_dir = os.path.join(BUILDS_DIR, config_name)

            src = os.path.join(BUILDS_DIR, config_name)
            dest = os.path.join(BINARIES_DIR, config_name)

            try:
                os.makedirs(dest)
            except OSError:
                pass

            # Build, tweaking paths to sdkconfig and sdkconfig.defaults
            ut_apply_config("ut-apply-config-" + config_name, ctx, build_args)

            build_target = base_actions["actions"]["all"]["callback"]

            build_target("all", ctx, build_args)

            # Copy artifacts to the output directory
            shutil.copyfile(
                os.path.join(build_args.project_dir, "sdkconfig"),
                os.path.join(dest, "sdkconfig"),
            )

            binaries = [PROJECT_NAME + x for x in [".elf", ".bin", ".map"]]

            for binary in binaries:
                shutil.copyfile(os.path.join(src, binary), os.path.join(dest, binary))

            try:
                os.mkdir(os.path.join(dest, "bootloader"))
            except OSError:
                pass

            shutil.copyfile(
                os.path.join(src, "bootloader", "bootloader.bin"),
                os.path.join(dest, "bootloader", "bootloader.bin"),
            )

            for partition_table in glob.glob(os.path.join(src, "partition_table", "partition-table*.bin")):
                try:
                    os.mkdir(os.path.join(dest, "partition_table"))
                except OSError:
                    pass
                shutil.copyfile(
                    partition_table,
                    os.path.join(dest, "partition_table", os.path.basename(partition_table)),
                )

            shutil.copyfile(
                os.path.join(src, "flasher_args.json"),
                os.path.join(dest, "flasher_args.json"),
            )

            binaries = glob.glob(os.path.join(src, "*.bin"))
            binaries = [os.path.basename(s) for s in binaries]

            for binary in binaries:
                shutil.copyfile(os.path.join(src, binary), os.path.join(dest, binary))

            gdbinit_src = os.path.join(src, "gdbinit")
            if os.path.exists(gdbinit_src):
                gdbinit_dest = os.path.join(dest, "gdbinit")
                if os.path.exists(gdbinit_dest):
                    shutil.rmtree(gdbinit_dest)
                shutil.copytree(gdbinit_src, gdbinit_dest)

    def ut_clean(ut_clean_name, ctx, args):
        config_name = re.match(r"ut-clean-(.*)", ut_clean_name).group(1)
        if config_name in CONFIG_NAMES:
            shutil.rmtree(os.path.join(BUILDS_DIR, config_name), ignore_errors=True)
            shutil.rmtree(os.path.join(BINARIES_DIR, config_name), ignore_errors=True)

    # Add global options
    extensions = {
        "global_options": [{
            "names": ["-T", "--test-components"],
            "help": "Specify the components to test.",
            "scope": "shared",
            "multiple": True,
        }, {
            "names": ["-E", "--test-exclude-components"],
            "help": "Specify the components to exclude from testing.",
            "scope": "shared",
            "multiple": True,
        }],
        "global_action_callbacks": [],
        "actions": {},
    }

    # This generates per-config targets (clean, build, apply-config).
    build_all_config_deps = []
    clean_all_config_deps = []

    for config in CONFIG_NAMES:
        config_build_action_name = "ut-build-" + config
        config_clean_action_name = "ut-clean-" + config
        config_apply_config_action_name = "ut-apply-config-" + config

        extensions["actions"][config_build_action_name] = {
            "callback":
            ut_build,
            "help":
            "Build unit-test-app with configuration provided in configs/NAME. " +
            "Build directory will be builds/%s/, " % config_build_action_name +
            "output binaries will be under output/%s/" % config_build_action_name,
        }

        extensions["actions"][config_clean_action_name] = {
            "callback": ut_clean,
            "help": "Remove build and output directories for configuration %s." % config_clean_action_name,
        }

        extensions["actions"][config_apply_config_action_name] = {
            "callback":
            ut_apply_config,
            "help":
            "Generates configuration based on configs/%s in sdkconfig file." % config_apply_config_action_name +
            "After this, normal all/flash targets can be used. Useful for development/debugging.",
        }

        build_all_config_deps.append(config_build_action_name)
        clean_all_config_deps.append(config_clean_action_name)

    extensions["actions"]["ut-build-all-configs"] = {
        "callback": ut_build,
        "help": "Build all configurations defined in configs/ directory.",
        "dependencies": build_all_config_deps,
    }

    extensions["actions"]["ut-clean-all-configs"] = {
        "callback": ut_clean,
        "help": "Remove build and output directories for all configurations defined in configs/ directory.",
        "dependencies": clean_all_config_deps,
    }

    return extensions


def add_argument_extensions(parser):
    pass

def add_action_extensions(base_functions, base_actions):
    PROJECT_NAME = "gen_ut_app"
    PROJECT_PATH = os.getcwd()

    # List of unit-test-app configurations.
    # Each file in configs/ directory defines a configuration. The format is the
    # same as sdkconfig file. Configuration is applied on top of sdkconfig.defaults
    # file from the project directory
    CONFIG_NAMES = os.listdir(os.path.join(PROJECT_PATH, "configs"))

    # Build (intermediate) and output (artifact) directories
    BUILDS_DIR = os.path.join(PROJECT_PATH, "builds")
    BINARIES_DIR = os.path.join(PROJECT_PATH, "output")

    def ut_apply_config(ut_apply_config_name, args):
        config_name = re.match(r"ut-apply-config-(.*)", ut_apply_config_name).group(1)

        def set_config_build_variables(prop, defval=None):
            property_value = re.findall(r"^%s=(.+)" % prop, config_file_content, re.MULTILINE)
            if (property_value):
                property_value = property_value[0]
            else:
                property_value = defval

            if (property_value):
                try:
                    args.define_cache_entry.append("%s=" % prop + property_value)
                except AttributeError:
                    args.define_cache_entry = ["%s=" % prop + property_value]

            return property_value

        sdkconfig_set = None

        if args.define_cache_entry:
            sdkconfig_set = filter(lambda s: "SDKCONFIG=" in s, args.define_cache_entry)

        sdkconfig_path = os.path.join(args.project_dir, "sdkconfig")

        if sdkconfig_set:
            sdkconfig_path = sdkconfig_set[-1].split("=")[1]
            sdkconfig_path = os.path.abspath(sdkconfig_path)

        try:
            os.remove(sdkconfig_path)
        except OSError:
            pass

        if config_name in CONFIG_NAMES:
            # Parse the sdkconfig for components to be included/excluded and tests to be run
            config = os.path.join(PROJECT_PATH, "configs", config_name)

            with open(config, "r", encoding="utf-8") as config_file:
                config_file_content = config_file.read()

                set_config_build_variables("EXCLUDE_COMPONENTS", "''")

                test_components = set_config_build_variables("TEST_COMPONENTS", "''")

                set_config_build_variables("TEST_EXCLUDE_COMPONENTS","''")

            with tempfile.NamedTemporaryFile(delete=False) as sdkconfig_temp:
                # Use values from the combined defaults and the values from
                # config folder to build config
                sdkconfig_default = os.path.join(PROJECT_PATH, "sdkconfig.defaults")

                with open(sdkconfig_default, "rb") as sdkconfig_default_file:
                    sdkconfig_temp.write(sdkconfig_default_file.read())

                sdkconfig_config = os.path.join(PROJECT_PATH, "configs", config_name)
                with open(sdkconfig_config, "rb") as sdkconfig_config_file:
                    sdkconfig_temp.write(b"\n")
                    sdkconfig_temp.write(sdkconfig_config_file.read())
            try:
                try:
                    args.define_cache_entry.append("SDKCONFIG_DEFAULTS=" + sdkconfig_temp.name)
                except AttributeError:
                    args.define_cache_entry = ["SDKCONFIG_DEFAULTS=" + sdkconfig_temp.name]

                reconfigure = base_functions["reconfigure"]
                reconfigure(None, args)
            finally:
                try:
                    os.unlink(sdkconfig_temp.name)
                except OSError:
                    pass
        else:
            if not config_name == "all-configs":
                print("unknown unit test app config for action '%s'" % ut_apply_config_name)

    # This target builds the configuration. It does not currently track dependencies,
    # but is good enough for CI builds if used together with clean-all-configs.
    # For local builds, use 'apply-config-NAME' target and then use normal 'all'
    # and 'flash' targets.
    def ut_build(ut_build_name, args):
        # Create a copy of the passed arguments to prevent arg modifications to accrue if
        # all configs are being built
        build_args = copy.copy(args)

        config_name = re.match(r"ut-build-(.*)", ut_build_name).group(1)

        if config_name in CONFIG_NAMES:
            build_args.build_dir = os.path.join(BUILDS_DIR, config_name)

            src = os.path.join(BUILDS_DIR, config_name)
            dest = os.path.join(BINARIES_DIR, config_name)

            try:
                os.makedirs(dest)
            except OSError:
                pass

            # Build, tweaking paths to sdkconfig and sdkconfig.defaults
            ut_apply_config("ut-apply-config-" + config_name, build_args)

            build_target = base_functions["build_target"]

            build_target("all", build_args)

            # Copy artifacts to the output directory
            shutil.copyfile(os.path.join(build_args.project_dir, "sdkconfig"), os.path.join(dest, "sdkconfig"))

            binaries = [PROJECT_NAME + x for x in [".elf", ".bin", ".map"]]

            for binary in binaries:
                shutil.copyfile(os.path.join(src, binary), os.path.join(dest, binary))

            try:
                os.mkdir(os.path.join(dest, "bootloader"))
            except OSError:
                pass

            shutil.copyfile(os.path.join(src, "bootloader", "bootloader.bin"), os.path.join(dest, "bootloader", "bootloader.bin"))

            for partition_table in glob.glob(os.path.join(src, "partition_table", "partition-table*.bin")):
                try:
                    os.mkdir(os.path.join(dest, "partition_table"))
                except OSError:
                    pass
                shutil.copyfile(partition_table, os.path.join(dest, "partition_table", os.path.basename(partition_table)))

            shutil.copyfile(os.path.join(src, "flasher_args.json"), os.path.join(dest, "flasher_args.json"))

            binaries = glob.glob(os.path.join(src, "*.bin"))
            binaries = [os.path.basename(s) for s in binaries]

            for binary in binaries:
                shutil.copyfile(os.path.join(src, binary), os.path.join(dest, binary))

            gdbinit_src = os.path.join(src, "gdbinit")
            if os.path.exists(gdbinit_src):
                gdbinit_dest = os.path.join(dest, "gdbinit")
                if os.path.exists(gdbinit_dest):
                    shutil.rmtree(gdbinit_dest)
                shutil.copytree(gdbinit_src, gdbinit_dest)

        else:
            if not config_name == "all-configs":
                print("unknown unit test app config for action '%s'" % ut_build_name)

    def ut_clean(ut_clean_name, args):
        config_name = re.match(r"ut-clean-(.*)", ut_clean_name).group(1)
        if config_name in CONFIG_NAMES:
            shutil.rmtree(os.path.join(BUILDS_DIR, config_name), ignore_errors=True)
            shutil.rmtree(os.path.join(BINARIES_DIR, config_name), ignore_errors=True)
        else:
            if not config_name == "all-configs":
                print("unknown unit test app config for action '%s'" % ut_clean_name)

    def ut_help(action, args):
        HELP_STRING = """
Additional unit-test-app specific targets

idf.py ut-build-NAME - Build unit-test-app with configuration provided in configs/NAME.
                    Build directory will be builds/NAME/, output binaries will be
                    under output/NAME/

idf.py ut-clean-NAME - Remove build and output directories for configuration NAME.

idf.py ut-build-all-configs - Build all configurations defined in configs/ directory.

idf.py ut-apply-config-NAME - Generates configuration based on configs/NAME in sdkconfig
                    file. After this, normal all/flash targets can be used.
                    Useful for development/debugging.
"""
        print(HELP_STRING)

    # Build dictionary of action extensions
    extensions = dict()

    # This generates per-config targets (clean, build, apply-config).
    build_all_config_deps = []
    clean_all_config_deps = []

    for config in CONFIG_NAMES:
        config_build_action_name = "ut-build-" + config
        config_clean_action_name = "ut-clean-" + config
        config_apply_config_action_name = "ut-apply-config-" + config

        extensions[config_build_action_name] = (ut_build, [], [])
        extensions[config_clean_action_name] = (ut_clean, [], [])
        extensions[config_apply_config_action_name] = (ut_apply_config, [], [])

        build_all_config_deps.append(config_build_action_name)
        clean_all_config_deps.append(config_clean_action_name)

    extensions["ut-build-all-configs"] = (ut_build, build_all_config_deps, [])
    extensions["ut-clean-all-configs"] = (ut_clean, clean_all_config_deps, [])

    extensions["ut-help"] = (ut_help, [], [])

    base_actions.update(extensions)
