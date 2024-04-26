<script lang="ts">
    import { PivotAngle, ShooterSpeed } from "../ntStores";
</script>

<main>
    <div class="driver-view-container">
        <div class="metric-container">
            <code class="metric">{($ShooterSpeed * 100).toFixed(2)}%</code>
            <p class="metric-description">Shooter Speed</p>
        </div>
        <div class="metric-container">
            <code class="metric">{($PivotAngle * (180 / Math.PI)).toFixed(2)}°</code>
            <p class="metric-description">Pivot Angle</p>
        </div>
    </div>
</main>

<style>
    .driver-view-container {
        display: flex;
        margin: 12.5% 20% ;
        height: 100%;
        flex-direction: row;
        justify-content: center;
        align-items: center;
    }

    .metric-container {
        display: flex;
        flex-direction: column;
        justify-content: center;
        align-items: center;
        flex-grow: 1;
    }

    .metric {
        margin: 0.8rem;
        padding: 1rem 2rem;
        height: fit-content;
        width: 7ch;
        border-radius: 2rem;
        font-size: 7em;
        font-weight: 800;
        background-color: var(--background-tertiary);
    }

    .metric-description {
        margin: 0;
        font-size: 2rem;
    }
</style>
