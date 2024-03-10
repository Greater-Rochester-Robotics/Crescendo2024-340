<script lang="ts">
    import {
        TunableNoteVelocity,
        TunableNormFudge,
        TunableStrafeFudge,
        TunableSpinCompensation,
        TunableSpeakerXFudge,
        TunableSpeakerYFudge,
    } from "../ntStores";

    const onNoteVelocityBlur = (e: FocusEvent) => {
        $TunableNoteVelocity = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };

    const onNormFudgeBlur = (e: FocusEvent) => {
        $TunableNormFudge = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };

    const onStrafeFudgeBlur = (e: FocusEvent) => {
        $TunableStrafeFudge = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };

    const onSpinCompensationBlur = (e: FocusEvent) => {
        $TunableSpinCompensation = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };

    const onSpeakerXBlur = (e: FocusEvent) => {
        $TunableSpeakerXFudge = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };

    const onSpeakerYBlur = (e: FocusEvent) => {
        $TunableSpeakerYFudge = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };
</script>

<main>
    <div class="tunable-container">
        <br />
        <p>Note Velocity:</p>
        <code contenteditable on:blur="{onNoteVelocityBlur}">{$TunableNoteVelocity}</code>
        <br /><br />
        <p>Norm Fudge:</p>
        <code contenteditable on:blur="{onNormFudgeBlur}">{$TunableNormFudge}</code>
        <br /><br />
        <p>Strafe Fudge:</p>
        <code contenteditable on:blur="{onStrafeFudgeBlur}">{$TunableStrafeFudge}</code>
        <br /><br />
        <p>Spin Compensation:</p>
        <code contenteditable on:blur="{onSpinCompensationBlur}">{$TunableSpinCompensation}</code>
        <br /><br />
        <p>Speaker X:</p>
        <code contenteditable on:blur="{onSpeakerXBlur}">{$TunableSpeakerXFudge}</code>
        <br /><br />
        <p>Speaker Y:</p>
        <code contenteditable on:blur="{onSpeakerYBlur}">{$TunableSpeakerYFudge}</code>
    </div>
</main>

<style>
    .tunable-container {
        font-size: 1.2rem;
    }

    .tunable-container p {
        font-weight: 700;
    }

    .tunable-container code {
        padding: 0.5rem;
        border-radius: 0.5rem;
        background-color: var(--background-tertiary);
        outline: 0 solid transparent;
    }

    .tunable-container code::selection {
        background-color: var(--accent-color);
    }

    .tunable-container code:focus {
        outline: 2px solid var(--accent-color);
    }
</style>
