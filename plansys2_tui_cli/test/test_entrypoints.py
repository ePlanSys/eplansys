import unittest
from importlib.metadata import entry_points


class TestConsoleEntryPoints(unittest.TestCase):
    def test_plansys2_tui_entrypoint_loads(self):
        eps = entry_points(group="console_scripts", name="plansys2_tui")
        ep = next(iter(eps), None)

        self.assertIsNotNone(
            ep,
            "Console script 'plansys2_tui' is not registered in console_scripts.",
        )
        self.assertEqual(
            ep.value,
            "plansys2_tui_cli.tui.app:run_app",
            "The 'plansys2_tui' entry point points to an unexpected target.",
        )

        try:
            loaded = ep.load()
        except Exception as exc:
            self.fail(
                "Failed to load the 'plansys2_tui' entry point: "
                f"{type(exc).__name__}: {exc}"
            )

        self.assertTrue(
            callable(loaded),
            "The loaded 'plansys2_tui' entry point is not callable.",
        )


class TestVerbEntryPoints(unittest.TestCase):
    def test_epistemic_verb_loads(self):
        """The verb must import without the epistemic messages being present.

        ros2cli loads every registered verb to build its help, so a verb that
        imported plansys2_epistemic_msgs at module scope would take the whole
        `ros2 plansys2` command down on a workspace built without it.
        """
        eps = entry_points(group="plansys2.verb", name="epistemic")
        ep = next(iter(eps), None)

        self.assertIsNotNone(
            ep,
            "The 'epistemic' verb is not registered in the plansys2.verb group.",
        )

        try:
            loaded = ep.load()
        except Exception as exc:
            self.fail(
                "Failed to load the 'epistemic' verb: "
                f"{type(exc).__name__}: {exc}"
            )

        self.assertTrue(
            hasattr(loaded, "add_arguments") and hasattr(loaded, "main"),
            "The 'epistemic' verb does not look like a VerbExtension.",
        )


if __name__ == "__main__":
    unittest.main()
