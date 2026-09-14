import time
from networktables import NetworkTables


if __name__ == "__main__":

    # =====================================================
    # CONNECT TO ROBOT
    # =====================================================

    NetworkTables.initialize(
        server='10.23.45.2'
    )


    # =====================================================
    # SMARTDASHBOARD
    # =====================================================

    sd = NetworkTables.getTable(
        "SmartDashboard"
    )


    print("Listening for Cobra values...")


    # =====================================================
    # CONTINUOUS DEBUG
    # =====================================================

    while True:

        # Raw values
        cobra0 = sd.getNumber(
            "Cobra 0",
            -1
        )

        cobra1 = sd.getNumber(
            "Cobra 1",
            -1
        )

        cobra2 = sd.getNumber(
            "Cobra 2",
            -1
        )

        cobra3 = sd.getNumber(
            "Cobra 3",
            -1
        )


        # Voltage values
        volt0 = sd.getNumber(
            "Cobra V0",
            -1
        )

        volt1 = sd.getNumber(
            "Cobra V1",
            -1
        )

        volt2 = sd.getNumber(
            "Cobra V2",
            -1
        )

        volt3 = sd.getNumber(
            "Cobra V3",
            -1
        )


        # Black tape status
        black = sd.getBoolean(
            "Black Tape",
            False
        )


        # Print everything
        print(
            f"C0={cobra0:.0f} ({volt0:.3f}V) | "
            f"C1={cobra1:.0f} ({volt1:.3f}V) | "
            f"C2={cobra2:.0f} ({volt2:.3f}V) | "
            f"C3={cobra3:.0f} ({volt3:.3f}V) | "
            f"Black={black}"
        )


        time.sleep(0.2)