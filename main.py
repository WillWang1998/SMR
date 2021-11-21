import collision_check
import smr

c_space = None


def build_c_space():
    global c_space
    obs_list = [[(0.0, 4.0), (3.0, 4.0), (3.0, 6.0), (0.0, 6.0)], [(7.0, 4.0), (7.0, 6.0), (10.0, 6.0), (10.0, 4.0)]]


    st = (2.0, 2.0)
    ed = (2.0, 8.0)

    needle_length = 1

    c_space = smr.ConfSpace(obs_list, st, ed, needle_length)


if __name__ == '__main__':
    print('py')
    # build_c_space()
    # build_smr()
    # TODO: solve_by_mdp()
