"""Result objects: a MotionPlan is an ordered list of Steps an executor can run and log."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field


def _r(v, n=1):
    return None if v is None else round(float(v), n)


@dataclass
class Check:
    """Margins of a pose or a move, metres (< 0 = violated). obstacles is +inf when nothing was in reach."""
    limits_ok: bool = True
    platform: float = float("inf")
    platform_mesh: float = float("inf")
    table: float = float("inf")
    obstacles: float = float("inf")
    worst_obstacle: str | None = None
    notes: list = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.limits_ok and self.platform >= 0 and self.platform_mesh >= 0 and self.table >= 0 and self.obstacles >= 0

    def merge(self, other: "Check") -> "Check":
        worse = other.obstacles < self.obstacles
        return Check(self.limits_ok and other.limits_ok, min(self.platform, other.platform),
                     min(self.platform_mesh, other.platform_mesh), min(self.table, other.table),
                     min(self.obstacles, other.obstacles), other.worst_obstacle if worse else self.worst_obstacle,
                     self.notes + other.notes)

    def to_dict(self) -> dict:
        cm = lambda v: None if v == float("inf") else round(v * 100, 2)   # noqa: E731
        return dict(ok=self.ok, limits_ok=self.limits_ok, platform_cm=cm(self.platform),
                    platform_mesh_cm=cm(self.platform_mesh), table_cm=cm(self.table),
                    obstacles_cm=cm(self.obstacles), worst_obstacle=self.worst_obstacle, notes=self.notes)


@dataclass
class Step:
    """One thing the arm does. kind 'move': J1-J5 go joint-linearly to `joints` with J6 at `j6`.
    kind 'gripper': only J6 goes to `j6` (`action` names the console button where one exists)."""
    kind: str
    name: str
    joints: list
    j6: float
    held: bool
    check: Check
    action: str | None = None
    note: str = ""

    @property
    def command(self) -> list[int]:
        return [round(v) for v in self.joints] + [round(self.j6)]

    def to_dict(self) -> dict:
        return dict(kind=self.kind, name=self.name, joints=[_r(v) for v in self.joints], j6=_r(self.j6),
                    command=self.command, held=self.held, action=self.action, note=self.note,
                    check=self.check.to_dict())


@dataclass
class MotionPlan:
    kind: str
    ok: bool
    steps: list = field(default_factory=list)
    summary: dict = field(default_factory=dict)
    reasons: list = field(default_factory=list)

    @property
    def end(self) -> list:
        return self.steps[-1].joints if self.steps else []

    def to_dict(self) -> dict:
        return dict(kind=self.kind, ok=self.ok, summary=self.summary, reasons=self.reasons,
                    steps=[s.to_dict() for s in self.steps])
