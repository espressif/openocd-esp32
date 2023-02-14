from .defs import *
from .utils import *
from .oocd import *
from .gdb import *
from .hw_specific.xtensa import *
from .hw_specific.esp import *

class NoMatchingClassError(RuntimeError):
    pass


def _look_for_subclasses(scope, chip_name, cls_obj):
    if scope is None:
        scope = globals()
    for a in scope:
        attr = scope[a]
        if type(attr) is type and issubclass(attr, cls_obj) and chip_name == getattr(attr, 'chip_name'):
            return attr
    return None

def _parse_target_triple(target_triple):
    # <arch><sub>-<vendor>-<sys>-<abi>
    arch = vendor = sys = abi = 'unknown'
    parts = target_triple.split('-')
    try:
        arch = parts.pop(0)
        vendor = parts.pop(0)
        sys = parts.pop(0)
        abi = parts.pop(0)
    except IndexError:
        pass
    return arch,vendor,sys,abi


def create_gdb(chip_name=None,
               target_triple=None,
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
    # At first, for backward compatibility try to get subclass for `chip_name`
    gdb_cls = _look_for_subclasses(scope, chip_name, Gdb)
    if gdb_cls:
        return gdb_cls(**gdb_init_args)
    if target_triple:
        # interpret `target_triple` as normal target triple like `xtensa-esp32s2-elf`
        arch,vendor,sys,_ = _parse_target_triple(target_triple)
        if sys == 'elf':
            if arch == 'xtensa':
                if vendor.startswith('esp'):
                    obj = GdbEspXtensa(**gdb_init_args)
                    obj.chip_name = chip_name
                    return obj
            elif arch.startswith('riscv32'):
                if vendor.startswith('esp'):
                    obj = GdbEspRiscv32(**gdb_init_args)
                    obj.chip_name = chip_name
                    return obj
            elif arch.startswith('riscv'):
                if vendor.startswith('esp'):
                    obj = GdbEspRiscv(**gdb_init_args)
                    obj.chip_name = chip_name
                    return obj

    raise NoMatchingClassError("GDB class was not found for chip '%s' and target tripple '%s'" % (chip_name, target_triple))


def create_oocd(chip_name=None,
                target_triple=None,
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
    # At first, for backward compatibility try to get subclass for `chip_name`
    oocd_cls = _look_for_subclasses(scope, chip_name, Oocd)
    if oocd_cls:
        return oocd_cls(**oocd_init_args)
    if target_triple:
        # interpret `target_triple` as normal target triple like `xtensa-esp32s2-elf`
        arch,vendor,sys,_ = _parse_target_triple(target_triple)
        if sys == 'elf':
            if arch == 'xtensa':
                if vendor.startswith('esp'):
                    obj = OocdEspXtensa(**oocd_init_args)
                    obj.chip_name = chip_name
                    return obj
            elif arch.startswith('riscv'):
                if vendor.startswith('esp'):
                    obj = OocdEspRiscv(**oocd_init_args)
                    obj.chip_name = chip_name
                    return obj
    raise NoMatchingClassError("OOCD class was not found for chip '%s' and target tripple '%s'" % (chip_name, target_triple))
