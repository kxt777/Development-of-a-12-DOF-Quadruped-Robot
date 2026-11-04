import math,unittest
from dog_navigation.discrete_actions import HeldAction,joystick_action,quantize,swept_clear,path_command


class DiscreteTests(unittest.TestCase):
    def test_six_actions_match_existing_joystick_circle_and_deadzone(self):
        expected=['forward','forward_left','turn_left','backward_left','backward','backward_right','turn_right','forward_right']
        for i,name in enumerate(expected):
            if i in (2,6):continue
            angle=i*math.pi/4
            x,y=-math.sin(angle),-math.cos(angle)
            def dead(v):return 0 if abs(v)<.08 else math.copysign((abs(v)-.08)/.92,v)
            a,v,w=joystick_action(i)
            self.assertEqual(a,name);self.assertAlmostEqual(v,-dead(y)*.08);self.assertAlmostEqual(w,-dead(x)*.15)
            self.assertEqual(quantize(v,w)[0],name)

    def test_20hz_alternation_does_not_change_action_inside_four_seconds(self):
        p=HeldAction();first=p.step(0,0,.15)
        for i in range(1,80):self.assertEqual(p.step(i*.05,0,.15*(-1)**i),first)
        self.assertEqual(p.step(4,0,-.15)[0],'forward_right')
        revision=p.revision
        self.assertEqual(p.step(8,0,-.15)[0],'forward_right');self.assertEqual(p.revision,revision)

    def test_slow_approach_keeps_translation_intent(self):
        self.assertEqual(quantize(.025,.15)[0],'forward_left')
        self.assertEqual(quantize(-.025,-.15)[0],'backward_right')

    def test_early_stop_does_not_allow_immediate_opposite_action(self):
        p=HeldAction();p.step(0,.08,0)
        self.assertEqual(p.step(1,0,0)[0],'stop')
        self.assertEqual(p.step(1.1,0,-.15)[0],'stop')
        self.assertEqual(p.step(4,0,-.15)[0],'forward_right')
        self.assertEqual(p.step(4.1,0,.15,interruption='obstacle')[0],'stop')
        self.assertEqual(p.step(5,0,.15)[0],'stop')

    def test_path_geometry_handles_behind_side_and_rotated_robot(self):
        self.assertEqual(quantize(*path_command([(0.,0.),(-1.,0.)],(0.,0.,0.)))[0],'backward')
        self.assertEqual(quantize(*path_command([(0.,0.),(1.,1.)],(0.,0.,0.)))[0],'forward_left')
        self.assertEqual(quantize(*path_command([(0.,0.),(-1.,1.)],(0.,0.,0.)))[0],'backward_right')
        self.assertEqual(quantize(*path_command([(0.,0.),(0.,1.)],(0.,0.,math.pi/2)))[0],'forward')
        for angle in range(-180,181):
            v,w=path_command([(0.,0.),(math.cos(math.radians(angle)),math.sin(math.radians(angle)))],(0.,0.,0.))
            self.assertNotEqual(quantize(v,w)[1],0.)

    def test_near_side_target_reverses_translation_without_undoing_turn(self):
        a=path_command([(0.,0.),(.1,.3)],(0.,0.,0.),approach_sign=1.)
        b=path_command([(0.,0.),(-.1,.3)],(0.,0.,0.),approach_sign=1.)
        self.assertGreater(a[0],0.);self.assertLess(b[0],0.)
        self.assertGreater(a[1],0.);self.assertGreater(b[1],0.)

    def test_held_sweep_checks_action_instead_of_current_nav_twist(self):
        grid=[0]*10000
        # 0.05m map, origin -2.5,-2.5; obstacle on the forward action.
        grid[50*100+55]=100
        self.assertFalse(swept_clear(grid,100,100,.05,(-2.5,-2.5,0),(0,0,0),.08,0,4))
        self.assertTrue(swept_clear(grid,100,100,.05,(-2.5,-2.5,0),(0,0,0),-.08,0,4))
        grid[50*100+55]=-1
        self.assertFalse(swept_clear(grid,100,100,.05,(-2.5,-2.5,0),(0,0,0),.08,0,4))

    def test_rotated_costmap_and_footprint_interior(self):
        grid=[0]*10000;grid[50*100+50]=100
        self.assertFalse(swept_clear(grid,100,100,.05,(2.5,-2.5,math.pi/2),(0,0,0),0,0,4))
        grid=[0]*10000;grid[50*100+52]=100
        polygon=[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]]
        self.assertTrue(swept_clear(grid,100,100,.05,(-2.5,-2.5,0),(0,0,0),0,0,4))
        self.assertFalse(swept_clear(grid,100,100,.05,(-2.5,-2.5,0),(0,0,0),0,0,4,polygon))


if __name__=='__main__':unittest.main()
