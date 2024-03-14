import { DEFAULT_URI } from "./constants";
import { NTSvelteClient } from "./lib/NTSvelte";

export const nt = new NTSvelteClient(DEFAULT_URI, { appName: `GRRDashboard` });

export const NTURI = nt.uriReadable();
export const NTConnected = nt.stateReadable();
export const NTBitrate = nt.bitrateReadable();
export const NTLatency = nt.latencyReadable();

export const RobotEnabled = nt.subscribe<boolean>(`/GRRDashboard/Robot/enabled`, false);
export const RobotMatchTime = nt.subscribe<number>(`/GRRDashboard/Robot/matchTime`, 0);
export const RobotBlueAlliance = nt.subscribe<boolean>(`/GRRDashboard/Robot/blueAlliance`, true);
export const RobotVoltage = nt.subscribe<number>(`/GRRDashboard/Robot/voltage`, 0);

export const AutosActive = nt.subscribe<string>(`/GRRDashboard/Autos/active`, ``);
export const AutosOptions = nt.subscribe<string[]>(`/GRRDashboard/Autos/options`, []);
export const AutosSelected = nt.publish<string>(`/GRRDashboard/Autos/selected`, `string`, ``);

export const FacingSpeaker = nt.subscribe<boolean>(`/GRRDashboard/Subsystems/Swerve/Details/facingSpeaker`, false);
export const PivotAtPosition = nt.subscribe<boolean>(`/GRRDashboard/Subsystems/Pivot/Details/atPosition`, false);
export const AtSpeed = nt.subscribe<boolean>(`/GRRDashboard/Subsystems/Shooter/Details/atSpeed`, false);
export const HasNote = nt.subscribe<boolean>(`/GRRDashboard/Subsystems/Pivot/Details/hasNote`, false);

export const RobotPosition = nt.subscribe<number[]>(`/GRRDashboard/Subsystems/Swerve/Visualizations/robot`, [0, 0, 0]);

export const TunableNoteVelocity = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableNoteVelocity`, `double`, 5.6);
export const TunableNormFudge = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableNormFudge`, `double`, 0.49);
export const TunableStrafeFudge = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableStrafeFudge`, `double`, 0.85);
export const TunableSpinCompensation = nt.publish<number>(
    `/GRRDashboard/Subsystems/Swerve/Details/tunableSpinCompensation`,
    `double`,
    -2 * (Math.PI / 180),
);
export const TunableDistanceFudge = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableDistanceFudge`, `double`, 0.0);
export const TunableSpeakerXFudge = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableSpeakerXFudge`, `double`, 0.0);
export const TunableSpeakerYFudge = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableSpeakerYFudge`, `double`, 0.0);
export const TunableAmpXFudge = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableAmpXFudge`, `double`, 0.0);
export const TunableAmpYFudge = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableAmpYFudge`, `double`, 0.0);

nt.connect();
