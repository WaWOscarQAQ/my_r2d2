use my_r2d2::interface_extractor::{Error, Extractor, Field, Interface, Kind};

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
