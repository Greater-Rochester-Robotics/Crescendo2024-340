<script lang="ts">
    import field24 from "../assets/field24.png";
    import { FIELD_HEIGHT, FIELD_WIDTH, ROBOT_SIZE } from "../constants";
    import { RobotBlueAlliance, HasNote, FacingSpeaker, PivotAtPosition, AtSpeed, RobotPosition } from "../ntStores";

    // import field22 from '../assets/field22.png';
    // import field23 from '../assets/field23.png';
    const field = field24;
</script>

<main>
    <div class="driver-view-container">
        <div class="status-container">
            <div class="subsystem-status-container">
                <div class="subsystem-status-{$HasNote ? `good` : `bad`}">
                    <p>Has Note</p>
                </div>
                <div class="subsystem-status-{$FacingSpeaker ? `good` : `bad`}">
                    <p>Facing Speaker</p>
                </div>
            </div>

            <div class="status-{$FacingSpeaker && $PivotAtPosition && $AtSpeed ? `good` : `bad`}">
                <p>Shoot!</p>
            </div>

            <div class="subsystem-status-container">
                <div class="subsystem-status-{$PivotAtPosition ? `good` : `bad`}">
                    <p>Pivot At Position</p>
                </div>
                <div class="subsystem-status-{$AtSpeed ? `good` : `bad`}">
                    <p>At Speed</p>
                </div>
            </div>
        </div>

        <div class="field">
            <img src="{field}" alt="field" />

            <svg
                viewBox="{0} {0} {FIELD_WIDTH} {FIELD_HEIGHT}"
                fill="none"
                stroke="var(--auto-line)"
                stroke-width="0.06"
                stroke-linecap="round"
                stroke-linejoin="round"
            >
                <g
                    transform="rotate({($RobotBlueAlliance ? Math.PI - $RobotPosition[2] : -$RobotPosition[2]) *
                        (180 / Math.PI)}, {($RobotBlueAlliance
                        ? $RobotPosition[0] - ROBOT_SIZE / 2
                        : FIELD_WIDTH - ($RobotPosition[0] + ROBOT_SIZE / 2)) +
                        ROBOT_SIZE / 2}, {$RobotBlueAlliance ? FIELD_HEIGHT - $RobotPosition[1] : $RobotPosition[1]})"
                >
                    <rect
                        x="{$RobotBlueAlliance ? $RobotPosition[0] - ROBOT_SIZE / 2 : FIELD_WIDTH - ($RobotPosition[0] + ROBOT_SIZE / 2)}"
                        y="{($RobotBlueAlliance ? FIELD_HEIGHT - $RobotPosition[1] : $RobotPosition[1]) - ROBOT_SIZE / 2}"
                        width="{ROBOT_SIZE}"
                        height="{ROBOT_SIZE}"
                    ></rect>
                    <circle
                        cx="{$RobotBlueAlliance ? $RobotPosition[0] - ROBOT_SIZE / 2 : FIELD_WIDTH - ($RobotPosition[0] + ROBOT_SIZE / 2)}"
                        cy="{$RobotBlueAlliance ? FIELD_HEIGHT - $RobotPosition[1] : $RobotPosition[1]}"
                        r="0.1"
                    ></circle>
                </g>
            </svg>
        </div>
    </div>
</main>

<style>
    .driver-view-container {
        display: flex;
        margin: 2rem;
        height: 100%;
        flex-direction: column;
        justify-content: center;
        align-items: center;
        gap: 3rem;
    }

    .status-container {
        display: flex;
        justify-content: center;
        width: 50%;
        gap: 1.5rem;
    }

    .subsystem-status-container {
        display: flex;
        flex-basis: 15%;
        flex-direction: column;
        justify-content: center;
        align-items: center;
        gap: 1rem;
    }

    .subsystem-status-good,
    .subsystem-status-bad {
        width: 100%;
        font-size: 1rem;
    }

    .status-good,
    .status-bad {
        display: flex;
        justify-content: center;
        align-items: center;
        flex-grow: 1;
        font-size: 3rem;
    }

    .status-good,
    .subsystem-status-good {
        background-color: var(--good);
        box-shadow: 0.3rem 0.3rem 0.5rem var(--background-shadow);
        transform: scale(1.02);
    }

    .status-bad,
    .subsystem-status-bad {
        background-color: var(--bad);
        box-shadow: 0.2rem 0.2rem 0.5rem var(--background-shadow);
    }

    .status-good,
    .status-bad,
    .subsystem-status-good,
    .subsystem-status-bad {
        color: var(--text-transparent-black);
        font-family: inherit;
        font-weight: 700;
        border: 3px solid transparent;
        border-radius: 1rem;
        transition:
            border-color 0.1s,
            transform 0.2s,
            box-shadow 0.2s;
    }

    .field {
        position: relative;
        display: inline-block;
    }

    .field img {
        display: block;
        height: 25rem;
    }
    .field svg {
        position: absolute;
        top: 0;
        left: 0;
    }
</style>
