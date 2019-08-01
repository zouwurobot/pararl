
from otter.experiment import run
import sys
if sys.path.count('/opt/ros/kinetic/lib/python2.7/dist-packages'):
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import otter.gym as gym

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
# import ROS.src.otter_kinova_grasping.otter_kinova_grasping.scripts.kinova_cup

env_params = {
      "environment_name": "ImageKinovaCupPusherEnv-v0",

    "random_init_cup_position": True,
    "random_target_position": True,

    "image": True,
    "image_dim": 48,

    'isRender': True
}

#env = gym.from_config(env_params)
do = 0#env.get_state_dim()
ds = 10
du = da = 0#env.get_action_dim()

horizon=50
experiment = dict(
    experiment_name='kinova-image',
    # experiment_type='myexp_real',
    experiment_type='myexp',
    env=env_params,
    model=dict(
        do=do, du=du, ds=ds, da=da, horizon=horizon,
        state_encoder=None,
        state_decoder=None,
        action_encoder=None,
        action_decoder=None,
        prior= None,
        cost=dict(cost_type='quadratic')
    ),
    control = dict(
        num_epochs=1000,
    ),
    train=dict(
        num_epochs=1000,
        learning_rate=1e-3,
        model_learning_rate=2e-5 * 50,
        beta_start=1e-4,
        beta_end=10.0,
        beta_rate=5e-5,
        beta_increase=0,
        batch_size=2,
        dump_every=100,
        summary_every=50,
    ),
    data=dict(
        num_rollouts=10,
        init_std=0.5,
        smooth_noise=False,
    ),
    dump_data=True,
    seed=0,
    out_dir='data/vae',
    horizon=100,

    rollouts_per_iter=20,
    num_iters=50,
    buffer_size=100,
    smooth_noise=False,
    num_videos=2,
    model_train={
        'num_epochs': 0
    }
)
run(experiment, remote=False, instance_type='m5.4xlarge',num_threads=1)


