<script lang="ts">
    import { TunableNoteVelocity, TunableNoteVelocityPub, TunableNormFudge, TunableNormFudgePub } from '../ntStores';

    const onNoteVelocityBlur = (e: FocusEvent) => {
        $TunableNoteVelocityPub = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };

    const onNormFudgeBlur = (e: FocusEvent) => {
        $TunableNormFudgePub = Number(decodeURIComponent((e.target as any)?.innerText).trim());;
    };
</script>

<main>
    <div class="tunable-container">
        <br>
        <p>Note Velocity:</p>
        <code contenteditable on:blur="{onNoteVelocityBlur}">{$TunableNoteVelocity}</code>
        <br><br>
    <p>Norm Fudge:</p>
    <code contenteditable on:blur="{onNormFudgeBlur}">{$TunableNormFudge}</code>
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
