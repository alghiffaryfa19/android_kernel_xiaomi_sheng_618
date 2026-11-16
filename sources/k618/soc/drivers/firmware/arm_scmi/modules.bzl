def register_modules(registry):
    registry.register(
        name = "drivers/firmware/arm_scmi/qcom_scmi_vendor",
        out = "qcom_scmi_vendor.ko",
        config = "CONFIG_QTI_SCMI_VENDOR_PROTOCOL",
        srcs = [
            # do not sort
            "drivers/firmware/arm_scmi/qcom_scmi_vendor.c",
        ],
    )

    registry.register(
        name = "drivers/firmware/arm_scmi/memlat_vendor",
        out = "memlat_vendor.ko",
        config = "CONFIG_QTI_SCMI_MEMLAT_PROTOCOL",
        srcs = [
            # do not sort
            "drivers/firmware/arm_scmi/memlat_vendor.c",
        ],
    )

    registry.register(
        name = "drivers/firmware/arm_scmi/pmu_vendor",
        out = "pmu_vendor.ko",
        config = "CONFIG_QTI_SCMI_PMU_PROTOCOL",
        srcs = [
            # do not sort
            "drivers/firmware/arm_scmi/pmu_vendor.c",
        ],
    )

    registry.register(
        name = "drivers/firmware/arm_scmi/c1dcvs_vendor",
        out = "c1dcvs_vendor.ko",
        config = "CONFIG_QTI_SCMI_C1DCVS_PROTOCOL",
        srcs = [
            # do not sort
            "drivers/firmware/arm_scmi/c1dcvs_vendor.c",
        ],
    )
