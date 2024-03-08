<script lang="ts">
    import {
        TunableNoteVelocity,
        TunableNoteVelocityPub,
        TunableNormFudge,
        TunableNormFudgePub,
        TunableSpeakerXFudge,
        TunableSpeakerXFudgePub,
        TunableSpeakerYFudge,
        TunableSpeakerYFudgePub,
        TunableAmpXFudge,
        TunableAmpXFudgePub,
        TunableAmpYFudge,
        TunableAmpYFudgePub,
    } from "../ntStores";

    const onNoteVelocityBlur = (e: FocusEvent) => {
        $TunableNoteVelocityPub = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };

    const onNormFudgeBlur = (e: FocusEvent) => {
        $TunableNormFudgePub = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };

    const onSpeakerXBlur = (e: FocusEvent) => {
        $TunableSpeakerXFudgePub = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };

    const onSpeakerYBlur = (e: FocusEvent) => {
        $TunableSpeakerYFudgePub = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };

    const onAmpXBlur = (e: FocusEvent) => {
        $TunableAmpXFudgePub = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };

    const onAmpYBlur = (e: FocusEvent) => {
        $TunableAmpYFudgePub = Number(decodeURIComponent((e.target as any)?.innerText).trim());
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
        <p>Speaker X:</p>
        <code contenteditable on:blur="{onSpeakerXBlur}">{$TunableSpeakerXFudge}</code>
        <br /><br />
        <p>Speaker Y:</p>
        <code contenteditable on:blur="{onSpeakerYBlur}">{$TunableSpeakerYFudge}</code>
        <br /><br />
        <p>Amp X:</p>
        <code contenteditable on:blur="{onAmpXBlur}">{$TunableAmpXFudge}</code>
        <br /><br />
        <p>Amp Y:</p>
        <code contenteditable on:blur="{onAmpYBlur}">{$TunableAmpYFudge}</code>
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
