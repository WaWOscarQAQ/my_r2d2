use my_r2d2::interface_extractor::{
    DataFile, Error, Extractor, Field, Interface, Kind, Primitive, TypeNode,
};

#[derive(Default)]
struct MockExtractor {
    items: Vec<Interface>,
}

impl MockExtractor {
    fn new(items: Vec<Interface>) -> Self {
        Self { items }
    }
}

impl Extractor for MockExtractor {
    fn extract(&self) -> Result<Vec<Interface>, Error> {
        Ok(self.items.clone())
    }
}

#[test]
fn mock_returns_interfaces() {
    let item = Interface::new(
        "/cmd_vel",
        Kind::Topic,
        vec![Field::new("linear_x", "float64")],
    );
    let ext = MockExtractor::new(vec![item.clone()]);

    assert_eq!(ext.extract().unwrap(), vec![item]);
}

#[test]
fn empty_mock_returns_empty_list() {
    assert!(MockExtractor::default().extract().unwrap().is_empty());
}

#[test]
fn from_str_maps_ros_primitive_names() {
    assert_eq!(TypeNode::from("float64"), TypeNode::Primitive(Primitive::F64));
    assert_eq!(TypeNode::from("string"), TypeNode::Primitive(Primitive::String));
}

#[test]
fn type_node_constructors_build_nested_and_array_shapes() {
    let ty = TypeNode::nested(vec![
        Field::new(
            "point",
            TypeNode::nested(vec![Field::new("x", "float64")]),
        ),
        Field::new("data", TypeNode::fixed_array(TypeNode::Primitive(Primitive::U8), 3)),
    ]);

    assert_eq!(
        ty,
        TypeNode::Nested(vec![
            Field::new(
                "point",
                TypeNode::Nested(vec![Field::new(
                    "x",
                    TypeNode::Primitive(Primitive::F64)
                )])
            ),
            Field::new(
                "data",
                TypeNode::Array(Box::new(TypeNode::Primitive(Primitive::U8)), Some(3))
            ),
        ])
    );
}

#[test]
fn interface_carries_associated_data_files() {
    let interface = Interface::new(
        "/cmd_vel",
        Kind::Topic,
        vec![Field::new("linear_x", "float64")],
    )
    .with_data_files(vec![DataFile::new(
        "geometry_msgs/msg/Twist.msg",
        "linear: Vector3",
    )]);

    assert_eq!(interface.data_files.len(), 1);
    assert_eq!(interface.data_files[0].name, "geometry_msgs/msg/Twist.msg");
    assert_eq!(interface.data_files[0].source, "linear: Vector3");
}
