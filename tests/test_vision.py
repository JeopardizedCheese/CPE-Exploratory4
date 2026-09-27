import copy
import json
import unittest
import cv2
import numpy as np
from vision import Detector, make_packet, warp
from sample_hsv import ranges_from_samples


def configuration():
    return {'hsv': {f'{i}_test': [{'lo':[h-5,80,45], 'hi':[h+5,255,255]}]
                    for i,h in enumerate([150,90,5,20,110,50],1)},
            'min_saturation':60, 'gem_area_mm2':{'min':350,'max':2500},
            'gem_area_px':{'min':80,'max':625},
            'arena':{'size_mm':[640,480],'mm_per_px':2,
                     'corners_px':[[0,0],[319,0],[319,239],[0,239]]},
            'vision':{'clearance_mm':30,'stable_frames':3}}


def color(h):
    return tuple(int(v) for v in cv2.cvtColor(np.uint8([[[h,220,190]]]),cv2.COLOR_HSV2BGR)[0,0])


class VisionTests(unittest.TestCase):
    def setUp(self):
        self.cfg = configuration()
        self.bg = np.full((240,320,3),140,np.uint8)
        self.frame = self.bg.copy()
        cv2.rectangle(self.frame,(130,100),(149,119),color(20),-1)

    def process(self, frame=None, cfg=None, bg=True, repeats=3):
        detector = Detector(cfg or self.cfg, self.bg if bg else None)
        for _ in range(repeats):
            result = detector.process(self.frame if frame is None else frame)
        return result

    def targets(self, result):
        return [o for o in result[1] if o.isolated and o.stable]

    def test_all_six_colors(self):
        for cid,h in enumerate([150,90,5,20,110,50],1):
            frame=self.bg.copy()
            cv2.rectangle(frame,(130,100),(149,119),color(h),-1)
            self.assertEqual(self.targets(self.process(frame))[0].color,cid)

    def test_highlight_keeps_one_stone(self):
        cv2.rectangle(self.frame,(135,100),(143,119),(255,255,255),-1)
        result=self.process()
        self.assertEqual(len(result[1]),1)
        self.assertEqual(self.targets(result)[0].color,4)

    def test_all_white_unknown(self):
        cv2.rectangle(self.frame,(130,100),(149,119),(255,255,255),-1)
        result=self.process()
        self.assertEqual(result[1][0].color,0)
        self.assertFalse(self.targets(result))

    def test_mixed_pile_rejected(self):
        cv2.rectangle(self.frame,(140,100),(149,119),color(90),-1)
        self.assertFalse(self.targets(self.process()))

    def test_unknown_obstacle_blocks_clearance(self):
        cv2.rectangle(self.frame,(157,100),(175,119),(255,255,255),-1)
        self.assertFalse(self.targets(self.process()))

    def test_large_robot_blocks(self):
        cv2.rectangle(self.frame,(155,60),(225,150),(20,20,20),-1)
        self.assertFalse(self.targets(self.process()))

    def test_background_zone_is_not_stone(self):
        cv2.rectangle(self.bg,(50,50),(90,90),color(20),-1)
        self.assertEqual(len(self.process(self.bg)[1]),0)

    def test_zone_exclusion(self):
        self.cfg['exclude_polygons']=[[[120,90],[160,90],[160,130],[120,130]]]
        self.assertFalse(self.process()[1])

    def test_near_wall_blocked(self):
        frame=self.bg.copy()
        cv2.rectangle(frame,(2,80),(21,99),color(20),-1)
        self.assertFalse(self.targets(self.process(frame)))

    def test_temporal_and_loss(self):
        d=Detector(self.cfg,self.bg)
        self.assertFalse(self.targets(d.process(self.frame)))
        d.process(self.frame)
        self.assertTrue(self.targets(d.process(self.frame)))
        self.assertFalse(self.targets(d.process(self.bg)))
        self.assertFalse(self.targets(d.process(self.frame)))

    def test_incomplete_setup_does_not_transmit_targets(self):
        for missing in ['background','hsv','arena']:
            cfg=copy.deepcopy(self.cfg)
            if missing=='hsv': cfg['hsv'].pop('6_test')
            if missing=='arena': cfg['arena']['corners_px']=[]
            result=self.process(cfg=cfg,bg=missing!='background')
            self.assertEqual(result[3],'setup_required')
            self.assertFalse(self.targets(result))

    def test_lighting_change_blocks(self):
        frame=np.full_like(self.bg,240)
        self.assertEqual(self.process(frame)[3],'lighting_or_camera_change')

    def test_small_exposure_drift(self):
        frame=np.clip(self.frame.astype(int)+10,0,255).astype(np.uint8)
        self.assertEqual(self.targets(self.process(frame))[0].color,4)

    def test_overlap_is_unknown(self):
        self.cfg['hsv']['5_test']=self.cfg['hsv']['4_test']
        self.assertFalse(self.targets(self.process()))

    def test_reference_shape_mismatch(self):
        with self.assertRaises(ValueError):
            Detector(self.cfg,self.bg[:20]).process(self.frame)

    def test_camera_resolution_change(self):
        self.cfg['arena']['source_size_px']=[640,480]
        with self.assertRaises(ValueError):
            self.process()

    def test_long_thin_cluster_rejected(self):
        frame=self.bg.copy()
        cv2.rectangle(frame,(100,100),(145,109),color(20),-1)
        self.assertFalse(self.targets(self.process(frame)))

    def test_shadowed_colored_face(self):
        frame=self.bg.copy()
        dark=cv2.cvtColor(np.uint8([[[20,220,90]]]),cv2.COLOR_HSV2BGR)[0,0]
        cv2.rectangle(frame,(130,100),(149,119),tuple(int(v) for v in dark),-1)
        self.assertEqual(self.targets(self.process(frame))[0].color,4)

    def test_severe_whitening_abstains(self):
        cv2.rectangle(self.frame,(134,100),(149,119),(255,255,255),-1)
        self.assertFalse(self.targets(self.process()))

    def test_packet_cap(self):
        result=self.process()
        packet=make_packet(result[1]*54,self.cfg,1,'123456789abc','ok',0)
        self.assertEqual(len(packet['targets']),8)
        self.assertLess(len(json.dumps(packet)),1400)

    def test_packet_units_and_stop(self):
        result=self.process()
        packet=make_packet(result[1],self.cfg,1,'123456789abc',result[3],0)
        self.assertEqual(packet['targets'][0]['x'],279)
        self.assertLess(len(json.dumps(packet)),1400)
        self.assertFalse(make_packet(result[1],self.cfg,2,'123456789abc','stopped',0)['targets'])

    def test_sampler_wrap_and_white(self):
        ranges=ranges_from_samples([[179,200,200]]*20+[[1,200,200]]*20)
        self.assertEqual(len(ranges),2)
        self.assertTrue(all(0<=r['hi'][0]<=179 for r in ranges))
        self.assertEqual(ranges_from_samples([[0,0,255]]*20),[])
        self.assertEqual(ranges_from_samples([]),[])


if __name__=='__main__':
    unittest.main()
