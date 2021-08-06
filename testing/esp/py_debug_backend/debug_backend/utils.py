import os

# This function needs to be called for paths passed to OOCD or GDB commands.
# API handles this automatically but ut should be used when if user composes commands himself, e.g. for Gdb.monitor_run().
# It makes paths portable across Windows and Linux versions of the tools.
def fixup_path(path):
    file_path = path
    if os.name == 'nt':
        # Convert filepath from Windows format if needed
        file_path = file_path.replace("\\","/");
    return file_path
