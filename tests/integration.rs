use newton::{Body, Bounds, Vec2, World};

fn bounds() -> Bounds {
    Bounds {
        min: Vec2::new(0.0, 0.0),
        max: Vec2::new(50.0, 50.0),
    }
}

#[test]
fn simulation_runs_many_steps_without_nan_or_panic() {
    let mut world = World::new(Vec2::new(0.0, -9.8), bounds());
    world.add_body(Body::new(Vec2::new(10.0, 40.0), 1.0, 1.0, 0.7));
    world.add_body(Body::new(Vec2::new(12.0, 30.0), 1.5, 2.0, 0.5));
    world.add_body(Body::new(Vec2::new(20.0, 20.0), 1.0, 1.0, 0.9));

    for _ in 0..2000 {
        world.step(1.0 / 60.0);
    }

    for body in &world.bodies {
        assert!(body.position.x.is_finite());
        assert!(body.position.y.is_finite());
        assert!(body.velocity.x.is_finite());
        assert!(body.velocity.y.is_finite());
        assert!(body.position.x >= bounds().min.x - 1e-6);
        assert!(body.position.x <= bounds().max.x + 1e-6);
        assert!(body.position.y >= bounds().min.y - 1e-6);
        assert!(body.position.y <= bounds().max.y + 1e-6);
    }
}

#[test]
fn deterministic_given_same_scene_and_steps() {
    let build = || {
        let mut world = World::new(Vec2::new(0.0, -9.8), bounds());
        world.add_body(Body::new(Vec2::new(10.0, 40.0), 1.0, 1.0, 0.7));
        world.add_body(Body::new(Vec2::new(12.0, 30.0), 1.5, 2.0, 0.5));
        world
    };

    let mut a = build();
    let mut b = build();

    for _ in 0..500 {
        a.step(1.0 / 60.0);
        b.step(1.0 / 60.0);
    }

    for (ba, bb) in a.bodies.iter().zip(b.bodies.iter()) {
        assert_eq!(ba.position.x, bb.position.x);
        assert_eq!(ba.position.y, bb.position.y);
    }
}
