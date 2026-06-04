
#include "../tools/test.h"

class ExampleTests : public test::TestCase {
public:
    void test_truth() {
        assert_true(true);
    }
};
