from dataclasses import dataclass


@dataclass
class DeliveryJob:
    pickup: str
    dropoff: str
    priority: int = 0
    requested_at: float = 0.0

    def label(self):
        return f"{self.pickup}->{self.dropoff}"


@dataclass
class ScheduledDeliveryJob:
    pickup: str
    dropoff: str
    trigger_time: float
    priority: int = 0
    dispatched: bool = False

    def label(self):
        return f"{self.pickup}->{self.dropoff}"


class JobMissionManager:
    def __init__(
        self,
        service_points,
        planner,
        default_pickup,
        default_dropoff,
        priority_distance_bonus=0.25,
    ):
        self.service_points = set(service_points)
        self.planner = planner
        self.priority_distance_bonus = priority_distance_bonus

        self.current = DeliveryJob(default_pickup, default_dropoff)
        self.pending = []
        self.completed = []

    def current_job(self):
        return self.current

    def pending_labels(self):
        return [job.label() for job in self.pending]

    def handle_incoming_job(self, pickup, dropoff, current_time, priority=0):
        pickup = pickup.upper()
        dropoff = dropoff.upper()

        if pickup not in self.service_points:
            return {
                "action": "reject",
                "job": f"{pickup}->{dropoff}",
                "reason": "unknown pickup point",
            }

        if dropoff not in self.service_points:
            return {
                "action": "reject",
                "job": f"{pickup}->{dropoff}",
                "reason": "unknown dropoff point",
            }

        if pickup == dropoff:
            return {
                "action": "reject",
                "job": f"{pickup}->{dropoff}",
                "reason": "pickup and dropoff are the same",
            }

        job = DeliveryJob(
            pickup=pickup,
            dropoff=dropoff,
            priority=max(0, int(priority)),
            requested_at=current_time,
        )

        if self.current is not None and self.current.label() == job.label():
            self.current.priority = max(self.current.priority, job.priority)
            return {
                "action": "merge_current",
                "job": job.label(),
                "priority": self.current.priority,
            }

        for pending_job in self.pending:
            if pending_job.label() == job.label():
                pending_job.priority = max(pending_job.priority, job.priority)
                return {
                    "action": "merge_pending",
                    "job": job.label(),
                    "priority": pending_job.priority,
                    "pending": self.pending_labels(),
                }

        if self.current is None:
            self.current = job
            return {
                "action": "accept",
                "job": job.label(),
                "priority": job.priority,
            }

        self.pending.append(job)
        return {
            "action": "queue",
            "job": job.label(),
            "priority": job.priority,
            "pending": self.pending_labels(),
        }

    def mark_current_delivered(self):
        delivered_job = self.current
        self.current = None

        if delivered_job is not None:
            self.completed.append(delivered_job)

        return delivered_job

    def select_next_job(self, current_node):
        if not self.pending:
            return None

        best_index = 0
        best_score = float("inf")

        for index, job in enumerate(self.pending):
            score = self._score_job(job, current_node)

            if score < best_score:
                best_score = score
                best_index = index

        self.current = self.pending.pop(best_index)
        return self.current

    def _score_job(self, job, current_node):
        pickup_route = self.planner.plan(current_node, job.pickup)
        dropoff_route = self.planner.plan(job.pickup, job.dropoff)
        distance = self.planner.path_distance(pickup_route)
        distance += self.planner.path_distance(dropoff_route)

        return distance - job.priority * self.priority_distance_bonus
