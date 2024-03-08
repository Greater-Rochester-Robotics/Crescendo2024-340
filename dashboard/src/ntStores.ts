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

export const TunableNoteVelocity = nt.subscribe<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableNoteVelocity`, 5);
export const TunableNoteVelocityPub = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableNoteVelocity`, `double`, 5);
export const TunableNormFudge = nt.subscribe<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableNormFudge`, 0.9);
export const TunableNormFudgePub = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableNormFudge`, `double`, 0.9);
export const TunableSpeakerXFudge = nt.subscribe<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableSpeakerXFudge`, 0.0);
export const TunableSpeakerXFudgePub = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableSpeakerXFudge`, `double`, 0.0);
export const TunableSpeakerYFudge = nt.subscribe<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableSpeakerYFudge`, 0.0);
export const TunableSpeakerYFudgePub = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableSpeakerYFudge`, `double`, 0.0);
export const TunableAmpXFudge = nt.subscribe<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableAmpXFudge`, 0.0);
export const TunableAmpXFudgePub = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableAmpXFudge`, `double`, 0.0);
export const TunableAmpYFudge = nt.subscribe<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableAmpYFudge`, 0.0);
export const TunableAmpYFudgePub = nt.publish<number>(`/GRRDashboard/Subsystems/Swerve/Details/tunableAmpYFudge`, `double`, 0.0);

nt.connect();
