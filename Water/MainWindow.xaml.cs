using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Media.Media3D;
using BulletSharp;
using HelixToolkit.Wpf;

namespace Water
{
    public partial class MainWindow : Window
    {
        // Основные переменные
        private List<ModelVisual3D> waterVoxels = new List<ModelVisual3D>();
        private const int MaxVoxels = 300;

        // Физический мир
        private DiscreteDynamicsWorld physicsWorld;
        private List<RigidBody> rigidBodies = new List<RigidBody>();

        // Таймер для обновления физики
        private System.Windows.Threading.DispatcherTimer physicsTimer;

        public MainWindow()
        {
            InitializeComponent();
            InitializeScene();
            InitializePhysics();
            StartSimulation();
            physicsWorld.DebugDrawWorld();
        }

        // Инициализация 3D-сцены
        private void InitializeScene()
        {
            // Бассейн (дно и стенки) - увеличен для наглядности
            var poolBottom = new RectangleVisual3D
            {
                Width = 4,
                Length = 4,
                Material = new DiffuseMaterial(System.Windows.Media.Brushes.DarkBlue),
                Transform = new TranslateTransform3D(0, 0, 1.5f) // Теперь Z=0 — это дно
            };
            MainViewport.Children.Add(poolBottom);

            // Стенки бассейна (теперь они по бокам вдоль Z)
            var poolWall1 = new BoxVisual3D
            {
                Width = 4,
                Height = 1.25,
                Length = 0.1,
                Material = MaterialHelper.CreateMaterial(System.Windows.Media.Brushes.Red),
                Transform = new TranslateTransform3D(-2, 0, 1.25) // Задняя стенка
            };
            MainViewport.Children.Add(poolWall1);

            var poolWall2 = new BoxVisual3D
            {
                Width = 0.1,
                Height = 1.25,
                Length = 4,
                Material = MaterialHelper.CreateMaterial(System.Windows.Media.Brushes.Green),
                Transform = new TranslateTransform3D(0, -2, 1.25) // Левая стенка
            };
            MainViewport.Children.Add(poolWall2);

            var poolWall3 = new BoxVisual3D
            {
                Width = 4,
                Height = 1.25,
                Length = 0.1,
                Material = MaterialHelper.CreateMaterial(System.Windows.Media.Brushes.Black),
                Transform = new TranslateTransform3D(2, 0, 1.25) // Правая стенка
            };
            MainViewport.Children.Add(poolWall3);

            var poolWall4 = new BoxVisual3D
            {
                Width = 0.1,
                Height = 1.25,
                Length = 4,
                Material = MaterialHelper.CreateMaterial(System.Windows.Media.Brushes.Green),
                Transform = new TranslateTransform3D(0, 2, 1.25) // Левая стенка
            };
            MainViewport.Children.Add(poolWall4);

            var pyramid = new TruncatedConeVisual3D
            {
                BaseRadius = 0.5,
                TopRadius = 0,
                Height = 1,
                Material = new DiffuseMaterial(System.Windows.Media.Brushes.Brown),
                Transform = new TranslateTransform3D(0, 0, 3) // Чуть выше дна (Z=0.1)
            };
            MainViewport.Children.Add(pyramid);

            var pipe = new TubeVisual3D
            {
                Path = new Point3DCollection { new Point3D(-2, 0, 8), new Point3D(0, 0, 6) }, // Теперь вода течет вниз по Z
                Diameter = 0.5,
                Material = new DiffuseMaterial(System.Windows.Media.Brushes.Gray)
            };

            var pipeRotation = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), 30), pipe.Path[1]);
            pipe.Transform = pipeRotation;
            MainViewport.Children.Add(pipe);


        }

        private void InitializePhysics()
        {
            // Настройка физического мира
            var collisionConfig = new DefaultCollisionConfiguration();
            var dispatcher = new CollisionDispatcher(collisionConfig);
            var broadphase = new DbvtBroadphase();
            var solver = new SequentialImpulseConstraintSolver();
            physicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
            physicsWorld.Gravity = new BulletSharp.Math.Vector3(0, 0, -9.8f); // Гравитация по Z


            // Дно бассейна
            var poolBottomShape = new BoxShape(2f, 2f, 0.1f); // Ширина/2, Длина/2, Толщина/2
            AddStaticObject(new BulletSharp.Math.Vector3(0, 0, 1.5f), poolBottomShape);

            // Задняя стенка (вертикально)
            var backWallShape = new BoxShape(0.1f, 2f, 1f); // Толщина по X, высота по Z, ширина по Y
            AddStaticObject(new BulletSharp.Math.Vector3(-2f, 0, 1.5f), backWallShape);

            // Левая стенка
            var leftWallShape = new BoxShape(2f, 0.1f, 2f); // Толщина по X, высота по Z, ширина по Y
            AddStaticObject(new BulletSharp.Math.Vector3(0, -2f, 1.5f), leftWallShape);

            // Правая стенка
            var rightWallShape = new BoxShape(2f, 0.1f, 2f); // Толщина по X, высота по Z, ширина по Y
            AddStaticObject(new BulletSharp.Math.Vector3(0, 2f, 1.5f), rightWallShape);

            // Передняя стенка
            var frontWallShape = new BoxShape(0.1f, 2f, 1f); // Толщина по X, высота по Z, ширина по Y
            AddStaticObject(new BulletSharp.Math.Vector3(2f, 0, 1.5f), frontWallShape);


            var pyramidShape = new ConeShape(0.5f, 1f);
            var pyramidTransform = BulletSharp.Math.Matrix.RotationX((float)Math.PI / 2) *
                                 BulletSharp.Math.Matrix.Translation(0, 0, 3.5f); // Центр масс


            var pyramidBodyInfo = new RigidBodyConstructionInfo(0, new DefaultMotionState(pyramidTransform),
                                 pyramidShape, BulletSharp.Math.Vector3.Zero);
            var pyramidBody = new RigidBody(pyramidBodyInfo);
            physicsWorld.AddRigidBody(pyramidBody);


            // Параметры симуляции
            physicsWorld.SolverInfo.NumIterations = 10; // Увеличить точность
            physicsWorld.DispatchInfo.AllowedCcdPenetration = 0.0001f;

            // Настройка CCD для динамических объектов (например, воды)
            foreach (var body in physicsWorld.CollisionObjectArray)
            {
                var rigidBody = body as RigidBody;
                if (rigidBody != null && rigidBody.CollisionFlags != CollisionFlags.StaticObject)
                {
                    rigidBody.CcdMotionThreshold = 0.01f;
                    rigidBody.CcdSweptSphereRadius = 0.05f;
                }
            }
        }

        // Добавление статического объекта в физический мир
        private void AddStaticObject(BulletSharp.Math.Vector3 position, CollisionShape shape)
        {
            var motionState = new DefaultMotionState(BulletSharp.Math.Matrix.Translation(position));
            var constructionInfo = new RigidBodyConstructionInfo(
                0,                  // Масса 0 = статический объект
                motionState,
                shape,
                BulletSharp.Math.Vector3.Zero);

            var body = new RigidBody(constructionInfo);

            // Включите все возможные коллизии
            body.CollisionFlags |= CollisionFlags.StaticObject;
            body.UserObject = "static"; // Для отладки

            physicsWorld.AddRigidBody(body,
                CollisionFilterGroups.StaticFilter,
                CollisionFilterGroups.DefaultFilter);
        }


        // Запуск симуляции
        private void StartSimulation()
        {
            physicsTimer = new System.Windows.Threading.DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(30)
            };
            physicsTimer.Tick += (s, e) =>
            {
                UpdatePhysics();
                GenerateWaterVoxel();
            };
            physicsTimer.Start();
        }


        private void GenerateWaterVoxel()
        {
            var sphere = new SphereVisual3D
            {
                Radius = 0.1, // Smaller radius
                Material = new DiffuseMaterial(System.Windows.Media.Brushes.Cyan),
                Transform = new TranslateTransform3D(0, 0, 6)
            };

            MainViewport.Children.Add(sphere);
            waterVoxels.Add(sphere);

            var shape = new SphereShape(0.1f);
            var mass = 0.01f;
            var localInertia = shape.CalculateLocalInertia(mass);

            // Было: new BulletSharp.Math.Vector3(-2, 2, 0)
            var motionState = new DefaultMotionState(new BulletSharp.Math.Matrix { Origin = new BulletSharp.Math.Vector3(0, 0, 6) });
            var body = new RigidBody(new RigidBodyConstructionInfo(mass, motionState, shape, localInertia));

            var rand = new Random();
            // Было: new BulletSharp.Math.Vector3(0, -1, 0)
            body.LinearVelocity = new BulletSharp.Math.Vector3((float)(0 + (rand.NextDouble()-0.5)*0.1), (float)(0 + (rand.NextDouble() - 0.5) * 0.1), -1); // Теперь вода летит вниз по Z
            physicsWorld.AddRigidBody(body);
            rigidBodies.Add(body);

            if (waterVoxels.Count > MaxVoxels)
            {
                var oldestVoxel = waterVoxels[0];
                MainViewport.Children.Remove(oldestVoxel);
                waterVoxels.RemoveAt(0);

                var oldestBody = rigidBodies[0];
                physicsWorld.RemoveRigidBody(oldestBody);
                rigidBodies.RemoveAt(0);
            }
        }


        private void UpdatePhysics()
        {
            physicsWorld.StepSimulation(1 / 60f, 10);

            for (int i = 0; i < rigidBodies.Count; i++)
            {
                var body = rigidBodies[i];
                if (body.MotionState != null)
                {
                    var transform = new BulletSharp.Math.Matrix();
                    body.MotionState.GetWorldTransform(out transform);

                    var origin = transform.Origin;
                    var visual = waterVoxels[i];
                    visual.Transform = new TranslateTransform3D(origin.X, origin.Y, origin.Z);
                }
            }
        }


    }
}