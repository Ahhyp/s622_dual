import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from c43_contact_safety import contact_violation,obb_overlap

class ContactSafety(unittest.TestCase):
    pose=[0,0,.275,0,0,0,1]
    supports=[dict(name='c43_support',center=[0,0,0],top_z=.25,radius=.04),dict(name='c43_destination',center=[0,.12,0],top_z=.25,radius=.04)]
    def verdict(self,name,point,phase='GRASP',source=True):
        block='position { '+' '.join(f'{a}: {v}' for a,v in zip('xyz',point))+' }'
        rec=dict(sample_contact_blocks=[dict(names=['c43_sample::base_link::collision',name],raw_block=block)])
        return contact_violation(rec,self.pose,phase,source,self.supports)
    def test_source_top_during_initial_grasp(self):
        self.assertEqual(self.verdict('c43_support::base_link::collision',[0,0,.25]),'')
    def test_source_contact_cannot_reappear_midcarry(self):
        self.assertTrue(self.verdict('c43_support::base_link::collision',[0,0,.25],'TRANSPORT',False))
    def test_support_side_is_not_top(self):
        self.assertTrue(self.verdict('c43_support::base_link::collision',[.04,0,.24]))
    def test_square_corner_not_inside_cylinder_disk(self):
        previous=self.pose;self.pose=[.099,.064,.275,0,0,0,1]
        self.assertTrue(self.verdict('c43_support::base_link::collision',[.039,.039,.25]))
        self.pose=previous
    def test_destination_contact_before_lower_is_rejected(self):
        self.assertTrue(self.verdict('c43_destination::base_link::collision',[0,.12,.25],'TRANSPORT'))
    def test_exact_finger_side(self):
        self.assertEqual(self.verdict('s622_dual_arm::right_finger2::right_finger2_collision',[0,.025,.275]),'')
    def test_finger_bottom_is_not_exempt(self):
        self.assertTrue(self.verdict('s622_dual_arm::right_finger2::right_finger2_collision',[0,0,.25]))
    def test_wrist_not_exempt(self):
        self.assertTrue(self.verdict('s622_dual_arm::right_wrist3_link::collision',[0,.025,.275]))
    def test_finger_impostor_not_exempt(self):
        self.assertTrue(self.verdict('obstacle::left_finger1::left_finger1_collision',[0,.025,.275]))
    def test_unknown_contact_rejected(self):
        self.assertTrue(self.verdict('c43_table::link::surface',[0,0,.25]))
    def test_missing_contact_geometry_fails_closed(self):
        with self.assertRaises(ValueError):contact_violation({},self.pose,'GRASP',True,self.supports)
    def test_finite_table_outside_edge_descent_allowed(self):
        self.assertFalse(obb_overlap([.82,0,-.01],np.eye(3),[.06,.025,.025],[0,0,-.015],np.eye(3),[.75,.4,.015]))
    def test_table_edge_penetration_detected(self):
        self.assertTrue(obb_overlap([.80,0,-.01],np.eye(3),[.06,.025,.025],[0,0,-.015],np.eye(3),[.75,.4,.015]))
    def test_above_table_clear(self):
        self.assertFalse(obb_overlap([0,0,.04],np.eye(3),[.06,.025,.025],[0,0,-.015],np.eye(3),[.75,.4,.015]))
    def test_nonfinite_geometry_rejected(self):
        with self.assertRaises(ValueError):obb_overlap([float('nan'),0,0],np.eye(3),[1,1,1],[0,0,0],np.eye(3),[1,1,1])

if __name__=='__main__':unittest.main()
