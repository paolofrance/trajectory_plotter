from bagpy import bagreader

if __name__ == '__main__':
    b = bagreader('/home/marie/cnr/paperi/miei/papers/gt_traj_fl/data/trial_1.bag')
    bmesg = b.message_by_topic('/alpha')