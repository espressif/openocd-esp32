import os

# ******* GDB **************************************
# Target states
TARGET_STATE_UNKNOWN = 0
TARGET_STATE_STOPPED = 1
TARGET_STATE_RUNNING = 2
# Target stop reasons
TARGET_STOP_REASON_UNKNOWN = 0
TARGET_STOP_REASON_SIGINT = 1
TARGET_STOP_REASON_SIGTRAP = 2
TARGET_STOP_REASON_BP = 3
TARGET_STOP_REASON_WP = 4
TARGET_STOP_REASON_WP_SCOPE = 5
TARGET_STOP_REASON_STEPPED = 6
TARGET_STOP_REASON_FN_FINISHED = 7

DEFAULT_GDB_INIT_SCRIPT_DIR = os.path.normpath(
    os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "hw_specific"
    )
)


class DebuggerError(RuntimeError):
    pass


class DebuggerTargetStateTimeoutError(DebuggerError):
    pass
