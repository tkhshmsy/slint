// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: GPL-3.0-only OR LicenseRef-Slint-Royalty-free-1.1 OR LicenseRef-Slint-commercial

/*!
This module contains the builtin Path related items.

When adding an item or a property, it needs to be kept in sync with different place.
Lookup the [`crate::items`] module documentation.
*/

use super::{FillRule, Item, ItemConsts, ItemRc, ItemRendererRef, RenderingResult};
use crate::graphics::{Brush, PathData, PathDataIterator};
use crate::input::{
    FocusEvent, FocusEventResult, InputEventFilterResult, InputEventResult, KeyEvent,
    KeyEventResult, MouseEvent,
};
use crate::item_rendering::CachedRenderingData;

use crate::layout::{LayoutInfo, Orientation};
use crate::lengths::{
    LogicalLength, LogicalPoint, LogicalRect, LogicalSize, LogicalVector, PointLengths,
};
#[cfg(feature = "rtti")]
use crate::rtti::*;
use crate::window::WindowAdapter;
use crate::{Coord, Property};
use alloc::rc::Rc;
use const_field_offset::FieldOffsets;
use core::pin::Pin;
use euclid::num::Zero;
use i_slint_core_macros::*;

/// The implementation of the `Path` element
#[repr(C)]
#[derive(FieldOffsets, Default, SlintElement)]
#[pin]
pub struct Path {
    pub x: Property<LogicalLength>,
    pub y: Property<LogicalLength>,
    pub width: Property<LogicalLength>,
    pub height: Property<LogicalLength>,
    pub elements: Property<PathData>,
    pub fill: Property<Brush>,
    pub fill_rule: Property<FillRule>,
    pub stroke: Property<Brush>,
    pub stroke_width: Property<LogicalLength>,
    pub viewbox_x: Property<f32>,
    pub viewbox_y: Property<f32>,
    pub viewbox_width: Property<f32>,
    pub viewbox_height: Property<f32>,
    pub clip: Property<bool>,
    pub cached_rendering_data: CachedRenderingData,
}

impl Item for Path {
    fn init(self: Pin<&Self>, _self_rc: &ItemRc) {}

    fn geometry(self: Pin<&Self>) -> LogicalRect {
        LogicalRect::new(
            LogicalPoint::from_lengths(self.x(), self.y()),
            LogicalSize::from_lengths(self.width(), self.height()),
        )
    }

    fn layout_info(
        self: Pin<&Self>,
        _orientation: Orientation,
        _window_adapter: &Rc<dyn WindowAdapter>,
    ) -> LayoutInfo {
        LayoutInfo { stretch: 1., ..LayoutInfo::default() }
    }

    fn input_event_filter_before_children(
        self: Pin<&Self>,
        event: MouseEvent,
        _window_adapter: &Rc<dyn WindowAdapter>,
        _self_rc: &ItemRc,
    ) -> InputEventFilterResult {
        if let Some(pos) = event.position() {
            if self.clip()
                && (pos.x < 0 as _
                    || pos.y < 0 as _
                    || pos.x_length() > self.width()
                    || pos.y_length() > self.height())
            {
                return InputEventFilterResult::Intercept;
            }
        }
        InputEventFilterResult::ForwardAndIgnore
    }

    fn input_event(
        self: Pin<&Self>,
        _: MouseEvent,
        _window_adapter: &Rc<dyn WindowAdapter>,
        _self_rc: &ItemRc,
    ) -> InputEventResult {
        InputEventResult::EventIgnored
    }

    fn key_event(
        self: Pin<&Self>,
        _: &KeyEvent,
        _window_adapter: &Rc<dyn WindowAdapter>,
        _self_rc: &ItemRc,
    ) -> KeyEventResult {
        KeyEventResult::EventIgnored
    }

    fn focus_event(
        self: Pin<&Self>,
        _: &FocusEvent,
        _window_adapter: &Rc<dyn WindowAdapter>,
        _self_rc: &ItemRc,
    ) -> FocusEventResult {
        FocusEventResult::FocusIgnored
    }

    fn render(
        self: Pin<&Self>,
        backend: &mut ItemRendererRef,
        self_rc: &ItemRc,
        size: LogicalSize,
    ) -> RenderingResult {
        let clip = self.clip();
        if clip {
            (*backend).save_state();
            (*backend).combine_clip(size.into(), LogicalLength::zero(), LogicalLength::zero());
        }
        (*backend).draw_path(self, self_rc, size);
        if clip {
            (*backend).restore_state();
        }
        RenderingResult::ContinueRenderingChildren
    }
}

impl Path {
    /// Returns an iterator of the events of the path and an offset, so that the
    /// shape fits into the width/height of the path while respecting the stroke
    /// width.
    pub fn fitted_path_events(self: Pin<&Self>) -> Option<(LogicalVector, PathDataIterator)> {
        let mut elements_iter = self.elements().iter()?;

        let stroke_width = self.stroke_width();
        let bounds_width = (self.width() - stroke_width).max(LogicalLength::zero());
        let bounds_height = (self.height() - stroke_width).max(LogicalLength::zero());
        let offset =
            LogicalVector::from_lengths(stroke_width / 2 as Coord, stroke_width / 2 as Coord);

        let viewbox_width = self.viewbox_width();
        let viewbox_height = self.viewbox_height();

        let maybe_viewbox = if viewbox_width > 0. && viewbox_height > 0. {
            Some(
                euclid::rect(self.viewbox_x(), self.viewbox_y(), viewbox_width, viewbox_height)
                    .to_box2d(),
            )
        } else {
            None
        };

        elements_iter.fit(bounds_width.get() as _, bounds_height.get() as _, maybe_viewbox);
        (offset, elements_iter).into()
    }
}

impl ItemConsts for Path {
    const cached_rendering_data_offset: const_field_offset::FieldOffset<Path, CachedRenderingData> =
        Path::FIELD_OFFSETS.cached_rendering_data.as_unpinned_projection();
}
