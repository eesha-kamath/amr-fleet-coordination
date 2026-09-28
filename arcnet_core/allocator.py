from abc import ABC, abstractmethod


class TaskAllocator(ABC):

    @abstractmethod
    def assign(self, tasks, robots):
        pass


class ManualAllocator(TaskAllocator):

    def assign(self, tasks, robots):
        return tasks


class CentralAllocator(TaskAllocator):

    def assign(self, tasks, robots):
        available = [
            r
            for r in robots
            if robots[r].active
        ]

        for task in tasks:
            if task.assigned_robot:
                continue

            if not available:
                break

            task.assigned_robot = available[0]

        return tasks


class ContractNetAllocator(TaskAllocator):

    def assign(self, tasks, robots):
        available = [
            r
            for r in robots
            if robots[r].active
        ]

        for task in tasks:
            if task.assigned_robot:
                continue

            if not available:
                break

            best = available[0]
            best_battery = robots[best].battery

            for robot_id in available:
                if robots[robot_id].battery > best_battery:
                    best = robot_id
                    best_battery = robots[robot_id].battery

            task.assigned_robot = best

        return tasks