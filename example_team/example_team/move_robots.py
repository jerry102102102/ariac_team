import asyncio

import rclpy
from rclpy.executors import Executor, MultiThreadedExecutor

from rclpy.time import Duration

from example_team.robot_interface import RobotInterface
from example_team.utils import AsyncUtils

async def spin_executor(executor: Executor, shutdown_event: asyncio.Event):
    while not shutdown_event.is_set():
        executor.spin_once(timeout_sec=1)
        await asyncio.sleep(0.0)  # yield control to asyncio loop

async def run():
    rclpy.init()

    executor = MultiThreadedExecutor()

    # Create robot nodes
    inspection_robot_1 = RobotInterface("inspection_robot_1")

    # # Add nodes to an executor 
    executor.add_node(inspection_robot_1)

    shutdown_event = asyncio.Event()
    spin_task = asyncio.create_task(spin_executor(executor, shutdown_event))

    try:
        # await AsyncUtils.await_for_duration(inspection_robot_1.get_clock(), Duration(seconds=2))
        await inspection_robot_1.ready()

        # plan = await inspection_robot_1.plan_to_named_configuration("home")

        # await inspection_robot_1.execute(plan)

    except Exception as e:
        print(5*'\n', e, 5*'\n')
    finally:
        shutdown_event.set()
        await spin_task  

def main():
    asyncio.run(run())