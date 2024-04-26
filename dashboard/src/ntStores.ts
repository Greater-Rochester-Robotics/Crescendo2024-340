import { DEFAULT_URI } from "./constants";
import { NTSvelteClient } from "./lib/NTSvelte";

export const nt = new NTSvelteClient(DEFAULT_URI, { appName: `GRRDashboard`, subscribers: { all: true, periodic: 20, saveHistory: false } });

export const NTURI = nt.uriReadable();
export const NTConnected = nt.stateReadable();
export const NTBitrate = nt.bitrateReadable();
export const NTLatency = nt.latencyReadable();

export const RobotEnabled = nt.subscribe<boolean>(`/GRRDashboard/Robot/enabled`, false);
export const RobotMatchTime = nt.subscribe<number>(`/GRRDashboard/Robot/matchTime`, 0);
export const RobotVoltage = nt.subscribe<number>(`/GRRDashboard/Robot/voltage`, 0);

export const PivotAngle = nt.subscribe<number>(`/GRRDashboard/Subsystems/Pivot/Details/target`, 0);
export const ShooterSpeed = nt.subscribe<number>(`/GRRDashboard/Subsystems/Shooter/Details/lastManualSpeed`, 0);

nt.connect();
