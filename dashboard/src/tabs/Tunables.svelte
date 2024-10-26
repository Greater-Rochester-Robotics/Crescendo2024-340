<script lang="ts">
    import {
        TunableDistanceFudge,
        TunableSpinCompensation
    } from "../ntStores";

    const onDistanceFudgeBlur = (e: FocusEvent) => {
        $TunableDistanceFudge = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };

    const onSpinCompensationBlur = (e: FocusEvent) => {
        $TunableSpinCompensation = Number(decodeURIComponent((e.target as any)?.innerText).trim());
    };
</script>

<main>
    <div class="tunable-container">
        <br />
        <p>Push Down:</p>
        <code contenteditable on:blur="{onDistanceFudgeBlur}">{$TunableDistanceFudge}</code>
        <br /><br />
        <p>Push Right:</p>
        <code contenteditable on:blur="{onSpinCompensationBlur}">{$TunableSpinCompensation}</code>
    </div>
</main>

<style>
    .tunable-container {
        font-size: 1.8rem;
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
