#!/usr/bin/env python3

import argparse


CTRL_STATES = {
    0: "INIT",
    1: "IDLE",
    2: "ENABLED",
    3: "FAULT",
}

STM_FAULTS = [
    (1 << 0, "ESTOP"),
    (1 << 1, "OC_LEFT"),
    (1 << 2, "OC_RIGHT"),
    (1 << 3, "STALL_LEFT"),
    (1 << 4, "STALL_RIGHT"),
    (1 << 5, "ENC_TIMEOUT_LEFT"),
    (1 << 6, "ENC_TIMEOUT_RIGHT"),
    (1 << 7, "ADC_STUCK"),
    (1 << 15, "GENERIC"),
]

COMM_FAULTS = [
    (1 << 0, "STARTUP_TIMEOUT_WAITING_FOR_WHEEL_STATE"),
    (1 << 1, "STALE_WHEEL_STATE"),
]

RESET_CAUSES = [
    (1 << 0, "BOR"),
    (1 << 1, "PIN"),
    (1 << 2, "POR/PDR"),
    (1 << 3, "SOFT"),
    (1 << 4, "IWDG"),
    (1 << 5, "WWDG"),
    (1 << 6, "LPWR"),
]


def parse_int(text):
    return int(text, 0)


def decode_bits(mask, table):
    names = [name for bit, name in table if mask & bit]
    known = 0
    for bit, _name in table:
        known |= bit
    unknown = mask & ~known
    if unknown:
        names.append(f"UNKNOWN_BITS_0x{unknown:x}")
    return names or ["none"]


def print_mask(label, value, table):
    names = decode_bits(value, table)
    print(f"{label}: {value} (0x{value:08x})")
    print(f"{label}_decoded: {', '.join(names)}")


def main():
    parser = argparse.ArgumentParser(description="Decode AMR STM and communication fault masks.")
    parser.add_argument("--fault-mask", type=parse_int, help="Value from /amr_stm/fault_mask")
    parser.add_argument("--safety-state", type=parse_int, help="Value from /amr_stm/safety_state")
    parser.add_argument("--comm-fault-mask", type=parse_int, help="Value from /amr_stm/comm_fault_mask")
    parser.add_argument("--reset-cause-mask", type=parse_int, help="Reset cause mask from /amr_stm/ros_diag index 18")
    args = parser.parse_args()

    printed = False
    if args.safety_state is not None:
        state = (args.safety_state >> 16) & 0xFFFF
        fault_mask = args.safety_state & 0xFFFF
        print(f"safety_state: {args.safety_state} (0x{args.safety_state:08x})")
        print(f"control_state: {CTRL_STATES.get(state, f'UNKNOWN_{state}')} ({state})")
        print_mask("fault_mask_from_safety_state", fault_mask, STM_FAULTS)
        printed = True
    if args.fault_mask is not None:
        print_mask("fault_mask", args.fault_mask, STM_FAULTS)
        printed = True
    if args.comm_fault_mask is not None:
        print_mask("comm_fault_mask", args.comm_fault_mask, COMM_FAULTS)
        printed = True
    if args.reset_cause_mask is not None:
        print_mask("reset_cause_mask", args.reset_cause_mask, RESET_CAUSES)
        printed = True
    if not printed:
        parser.print_help()
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
