import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='Baxter-v0',
    entry_point='gym_baxter.envs:BaxterEnv',
    timestep_limit=1000,
    reward_threshold=1.0,
    nondeterministic = True,
)

#register(
#    id='BaxterEmptyGoal-v0',
#    entry_point='gym_baxter.envs:SoccerEmptyGoalEnv',
#    timestep_limit=1000,
#    reward_threshold=10.0,
#    nondeterministic = True,
#)

