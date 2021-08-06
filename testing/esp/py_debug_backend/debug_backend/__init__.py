from .defs import *
from .utils import *
from .oocd import *
from .gdb import *
from .hw_specific.xtensa import *
from .hw_specific.esp import *

def _look_for_subclasses(scope, chip_name, cls_obj):
    if scope is None:
        scope = globals()
    for a in scope:
        attr = scope[a]
        if type(attr) is type and issubclass(attr, cls_obj) and chip_name == getattr(attr, 'chip_name'):
            return attr
    raise RuntimeError("Subclass of '%s' was not found for chip '%s'" % (cls_obj, chip_name))


def create_gdb(chip_name=None,
               gdb_path=None,
               remote_target=None,
               extended_remote_mode=None,
               gdb_log_file=None,
               log_level=None,
               log_stream_handler=None,
               log_file_handler=None,
               scope=None):
    """
        Creates GDB instance for specified chip name.
        See Gdb.__init__() for undocumented parameters.

        scope : dict
            Dictionary representing symbol table to look for GDB classes.
            By default the scope is limited to this package.

        Returns
        -------
        Gdb instance for specified chip name
    """
    gdb_init_args = {}
    if gdb_path is not None:
        gdb_init_args['gdb_path'] = gdb_path
    if remote_target is not None:
        gdb_init_args['remote_target'] = remote_target
    if extended_remote_mode is not None:
        gdb_init_args['extended_remote_mode'] = extended_remote_mode
    if gdb_log_file is not None:
        gdb_init_args['gdb_log_file'] = gdb_log_file
    if log_level is not None:
        gdb_init_args['log_level'] = log_level
    if log_stream_handler is not None:
        gdb_init_args['log_stream_handler'] = log_stream_handler
    if log_file_handler is not None:
        gdb_init_args['log_file_handler'] = log_file_handler
    gdb_cls = _look_for_subclasses(scope, chip_name, Gdb)
    return gdb_cls(**gdb_init_args)


def create_oocd(chip_name=None,
                oocd_exec=None,
                oocd_scripts=None,
                oocd_cfg_files=[],
                oocd_cfg_cmds=[],
                oocd_debug=2,
                oocd_args=[],
                host='localhost',
                log_level=None,
                log_stream_handler=None,
                log_file_handler=None,
                scope=None):
    """
        Creates OOCD instance for specified chip name.
        See Oocd.__init__() for undocumented parameters.

        scope : dict
            Dictionary representing symbol table to look for OOCD classes.
            By default the scope is limited to this package.

        Returns
        -------
        OOCD instance for specified chip name
    """

    oocd_init_args = {}
    if oocd_exec is not None:
        oocd_init_args['oocd_exec'] = oocd_exec
    if oocd_scripts is not None:
        oocd_init_args['oocd_scripts'] = oocd_scripts
    if oocd_cfg_files is not None:
        oocd_init_args['oocd_cfg_files'] = oocd_cfg_files
    if oocd_cfg_cmds is not None:
        oocd_init_args['oocd_cfg_cmds'] = oocd_cfg_cmds
    if oocd_debug is not None:
        oocd_init_args['oocd_debug'] = oocd_debug
    if oocd_args is not None:
        oocd_init_args['oocd_args'] = oocd_args
    if host is not None:
        oocd_init_args['host'] = host
    if log_level is not None:
        oocd_init_args['log_level'] = log_level
    if log_stream_handler is not None:
        oocd_init_args['log_stream_handler'] = log_stream_handler
    if log_file_handler is not None:
        oocd_init_args['log_file_handler'] = log_file_handler
    oocd_cls = _look_for_subclasses(scope, chip_name, Oocd)
    return oocd_cls(**oocd_init_args)
