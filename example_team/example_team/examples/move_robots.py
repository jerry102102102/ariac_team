import asyncio

import rclpy
from rclpy.executors import MultiThreadedExecutor

from example_team.environment_interface import Environment
from example_team.robot_interface import Robot
from example_team.utils import AsyncUtils

async def run():
    rclpy.init()

    # Create and spin executor
    executor = MultiThreadedExecutor()
    shutdown_event = asyncio.Event()
    spin_task = asyncio.create_task(AsyncUtils.spin_executor(executor, shutdown_event))

    environment = Environment()
    executor.add_node(environment)

    # Create robot nodes
    robots = {name: Robot(name) for name in ['inspection_robot_1', 'inspection_robot_2', 'assembly_robot_1', 'assembly_robot_2', 'gantry_welder']}

    # # Add nodes to an executor 
    for robot in robots.values():
        executor.add_node(robot)

    try:
        # Wait for competition to be ready
        environment.get_logger().info("Waiting for environment to be ready")
        await environment.competition.ready.wait()

        for robot in robots.values():
            robot.get_logger().info(f"Waiting for {robot.robot_name} to be ready")
            await robot.ready()

        # Move all robots up and down on a loop
        for i in range(20):
            tasks: list[asyncio.Task] = []
            direction = -1 if i % 2 == 0 else 1
            for name, robot in robots.items():
                pose = robot.current_pose
                if name == "gantry_welder":
                    pose.position.x += direction * 0.05
                else:
                    pose.position.z += direction * 0.1
                
                plan = await robot.plan_to_pose(pose, linear=True, vsf=0.5, asf=0.5)
                tasks.append(asyncio.create_task(robot.execute(plan)))

            await asyncio.wait(tasks, return_when=asyncio.ALL_COMPLETED)

            for task in tasks:
                exception = task.exception()
                
                if exception is not None:             
                    raise exception

    except Exception as e:
        print(5*'\n', e, 5*'\n')
    finally:
        shutdown_event.set()
        await spin_task  

def main():
    asyncio.run(run())