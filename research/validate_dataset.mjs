import fs from "node:fs";
import path from "node:path";

const dataDir = path.resolve("research/data");
const races = [];
const unconfirmed = [];
const excluded = [];
const sources = [];
const classifications = [];

for (const file of fs.readdirSync(dataDir).filter((name) => name.endsWith(".json")).sort()) {
  const data = JSON.parse(fs.readFileSync(path.join(dataDir, file), "utf8"));
  races.push(...(data.race_instances ?? []));
  unconfirmed.push(...(data.unconfirmed_candidates ?? []));
  excluded.push(...(data.excluded_items ?? []));
  sources.push(...(data.sources ?? []));
  for (const group of data.classification_groups ?? []) {
    for (const raceId of group.race_ids ?? []) {
      classifications.push({
        race_id: raceId,
        project: group.project,
        source_class: group.source_class,
        source_class_rule: group.source_class_rule,
        self_created_thread_evidence: group.self_created_thread_evidence,
        source_class_notes: group.source_class_notes,
      });
    }
  }
  for (const cluster of data.race_clusters ?? []) {
    for (const instance of cluster.instances ?? []) races.push({ ...(cluster.common ?? {}), ...instance });
  }
}

const errors = [];
const allowed = {
  confirmation_grade: new Set(["A", "B", "C"]),
  confirmed_main: new Set(["yes"]),
  strict_data_race: new Set(["yes", "no", "uncertain"]),
  root_cause_category: new Set(["R1", "R2", "R3", "R4", "R5", "R6", "R7"]),
  callback_race: new Set(["yes", "no", "uncertain"]),
  callback_relation: new Set(["direct", "callback_thread", "callback_lifecycle", "callback_indirect", "not_callback", "uncertain"]),
  fix_status: new Set(["unfixed", "pr_pending", "pr_closed_unmerged", "pr_merged", "commit_no_pr", "unknown"]),
  confidence: new Set(["high", "medium", "low"]),
};
const required = [
  "race_id", "project", "repository_url", "initial_report_date", "primary_source_url",
  "confirmation_grade", "confirmed_main", "strict_data_race", "root_cause_category",
  "callback_race", "fix_status", "confidence",
];
const seen = new Set();
const selectedProjects = new Set([
  "ros2_control", "Autoware Universe", "MoveIt 2", "SLAM Toolbox", "robot_localization",
  "rmf_ros2", "image_pipeline", "RTAB-Map", "Navigation2", "rclcpp", "rosbag2",
]);
const allowedSourceClasses = new Set(["lifecycle", "worker_thread", "callback", "others"]);
const classificationByRace = new Map();

for (const classification of classifications) {
  if (classificationByRace.has(classification.race_id)) errors.push(`${classification.race_id}: duplicate source_class`);
  classificationByRace.set(classification.race_id, classification);
  if (!allowedSourceClasses.has(classification.source_class)) errors.push(`${classification.race_id}: invalid source_class=${classification.source_class}`);
  for (const field of ["project", "source_class_rule", "self_created_thread_evidence", "source_class_notes"]) {
    if (!classification[field]) errors.push(`${classification.race_id}: missing classification ${field}`);
  }
}

for (const race of races) {
  for (const field of required) if (!race[field]) errors.push(`${race.race_id ?? "<missing-id>"}: missing ${field}`);
  if (seen.has(race.race_id)) errors.push(`${race.race_id}: duplicate race_id`);
  seen.add(race.race_id);
  if (race.initial_report_date < "2020-01-01") errors.push(`${race.race_id}: before time range`);
  if (!String(race.primary_source_url).startsWith("https://")) errors.push(`${race.race_id}: invalid source URL`);
  for (const [field, values] of Object.entries(allowed)) {
    if (race[field] !== undefined && !values.has(race[field])) errors.push(`${race.race_id}: invalid ${field}=${race[field]}`);
  }
  if (race.callback_race === "yes" && [undefined, "not_callback"].includes(race.callback_relation)) {
    errors.push(`${race.race_id}: callback=yes without callback relation`);
  }
  if (race.callback_race === "no" && race.callback_relation !== "not_callback") {
    errors.push(`${race.race_id}: callback=no must use not_callback`);
  }
  if (race.strict_data_race === "yes" && !race.strict_reason) errors.push(`${race.race_id}: strict=yes without strict_reason`);
  const classification = classificationByRace.get(race.race_id);
  if (selectedProjects.has(race.project)) {
    if (!classification) errors.push(`${race.race_id}: selected project missing source_class`);
    if (classification && classification.project !== race.project) errors.push(`${race.race_id}: source_class project mismatch`);
  }
}

for (const classification of classifications) {
  if (!seen.has(classification.race_id)) errors.push(`${classification.race_id}: source_class references unknown race_id`);
}

const byProject = new Map();
for (const race of races.filter((row) => row.confirmed_main === "yes")) {
  if (!byProject.has(race.project)) byProject.set(race.project, []);
  byProject.get(race.project).push(race);
}

const summary = [];
for (const [project, rows] of [...byProject.entries()].sort()) {
  const strictRows = rows.filter((r) => r.strict_data_race === "yes");
  const roots = Object.fromEntries([...allowed.root_cause_category].map((root) => [root, strictRows.filter((r) => r.root_cause_category === root).length]));
  const rootTotal = Object.values(roots).reduce((a, b) => a + b, 0);
  if (rootTotal !== strictRows.length) errors.push(`${project}: strict root total ${rootTotal} != strict ${strictRows.length}`);
  summary.push({
    project,
    confirmed_candidates: rows.length,
    strict: strictRows.length,
    strict_callback_involvement: strictRows.filter((r) => r.callback_race === "yes").length,
    ...(selectedProjects.has(project) ? Object.fromEntries([...allowedSourceClasses].map((sourceClass) => [`strict_source_${sourceClass}`, strictRows.filter((r) => classificationByRace.get(r.race_id)?.source_class === sourceClass).length])) : {}),
    ...roots,
  });
  if (selectedProjects.has(project)) {
    const sourceTotal = strictRows.filter((r) => classificationByRace.has(r.race_id)).length;
    if (sourceTotal !== strictRows.length) errors.push(`${project}: strict source-class total ${sourceTotal} != strict ${strictRows.length}`);
  }
}

console.log(JSON.stringify({ counts: { races: races.length, unconfirmed: unconfirmed.length, excluded: excluded.length, sources: sources.length, classifications: classifications.length }, summary, errors }, null, 2));
if (errors.length) process.exitCode = 1;
