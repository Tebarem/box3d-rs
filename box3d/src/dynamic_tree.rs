use std::{
    any::Any,
    collections::HashSet,
    ffi::{c_void, CString},
    panic::{catch_unwind, resume_unwind, AssertUnwindSafe},
    path::Path,
    sync::atomic::{AtomicU64, Ordering},
};

use box3d_sys as sys;

use crate::{
    math::{Aabb, Vec3},
    query::QueryStats,
    Error, Result,
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TreeProxy {
    id: i32,
    tree_id: u64,
}

impl TreeProxy {
    pub const fn id(self) -> i32 {
        self.id
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TreeHit {
    pub proxy: TreeProxy,
    pub user_data: u64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TreeClosestHit {
    pub distance_sqr_min: f32,
    pub proxy: TreeProxy,
    pub user_data: u64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TreeRayCastInput {
    pub origin: Vec3,
    pub translation: Vec3,
    pub max_fraction: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TreeBoxCastInput {
    pub aabb: Aabb,
    pub translation: Vec3,
    pub max_fraction: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TreeCastHit<T> {
    pub input: T,
    pub proxy: TreeProxy,
    pub user_data: u64,
}

pub struct DynamicTree {
    raw: sys::b3DynamicTree,
    proxies: HashSet<i32>,
    tree_id: u64,
}

impl DynamicTree {
    pub fn new(proxy_capacity: i32) -> Self {
        Self::try_new(proxy_capacity).expect("box3d returned an invalid dynamic tree")
    }

    pub fn try_new(proxy_capacity: i32) -> Result<Self> {
        if proxy_capacity < 0 {
            return Err(Error::InvalidInput);
        }

        Ok(Self {
            raw: unsafe { sys::b3DynamicTree_Create(proxy_capacity) },
            proxies: HashSet::new(),
            tree_id: next_tree_id(),
        })
    }

    pub fn create_proxy(&mut self, aabb: Aabb, category_bits: u64, user_data: u64) -> TreeProxy {
        assert_valid_aabb(aabb);
        let proxy = TreeProxy {
            id: unsafe {
                sys::b3DynamicTree_CreateProxy(&mut self.raw, aabb.into(), category_bits, user_data)
            },
            tree_id: self.tree_id,
        };
        self.proxies.insert(proxy.id);
        proxy
    }

    pub fn proxy_from_id(&self, id: i32) -> Result<TreeProxy> {
        if self.proxies.contains(&id) {
            Ok(TreeProxy {
                id,
                tree_id: self.tree_id,
            })
        } else {
            Err(Error::InvalidInput)
        }
    }

    pub fn destroy_proxy(&mut self, proxy: TreeProxy) -> Result<()> {
        let proxy_id = self.proxy_id(proxy)?;
        unsafe { sys::b3DynamicTree_DestroyProxy(&mut self.raw, proxy_id) };
        self.proxies.remove(&proxy_id);
        Ok(())
    }

    pub fn move_proxy(&mut self, proxy: TreeProxy, aabb: Aabb) -> Result<()> {
        assert_valid_aabb(aabb);
        let proxy_id = self.proxy_id(proxy)?;
        unsafe { sys::b3DynamicTree_MoveProxy(&mut self.raw, proxy_id, aabb.into()) };
        Ok(())
    }

    pub fn enlarge_proxy(&mut self, proxy: TreeProxy, aabb: Aabb) -> Result<()> {
        assert_valid_aabb(aabb);
        let proxy_id = self.proxy_id(proxy)?;
        unsafe { sys::b3DynamicTree_EnlargeProxy(&mut self.raw, proxy_id, aabb.into()) };
        Ok(())
    }

    pub fn set_category_bits(&mut self, proxy: TreeProxy, category_bits: u64) -> Result<()> {
        let proxy_id = self.proxy_id(proxy)?;
        unsafe { sys::b3DynamicTree_SetCategoryBits(&mut self.raw, proxy_id, category_bits) };
        Ok(())
    }

    pub fn category_bits(&mut self, proxy: TreeProxy) -> Result<u64> {
        let proxy_id = self.proxy_id(proxy)?;
        Ok(unsafe { sys::b3DynamicTree_GetCategoryBits(&mut self.raw, proxy_id) })
    }

    pub fn user_data(&self, proxy: TreeProxy) -> Result<u64> {
        let proxy_id = self.proxy_id(proxy)?;
        Ok(unsafe {
            (*self.raw.nodes.add(proxy_id as usize))
                .__bindgen_anon_1
                .userData
        })
    }

    pub fn aabb(&self, proxy: TreeProxy) -> Result<Aabb> {
        let proxy_id = self.proxy_id(proxy)?;
        Ok(unsafe { (*self.raw.nodes.add(proxy_id as usize)).aabb }.into())
    }

    pub fn query<F>(
        &self,
        aabb: Aabb,
        mask_bits: u64,
        require_all_bits: bool,
        mut f: F,
    ) -> QueryStats
    where
        F: FnMut(TreeHit) -> bool,
    {
        assert_valid_aabb(aabb);
        let mut context = QueryContext {
            f: &mut f,
            panic: None,
            tree_id: self.tree_id,
        };
        let stats = unsafe {
            sys::b3DynamicTree_Query(
                &self.raw,
                aabb.into(),
                mask_bits,
                require_all_bits,
                Some(query_callback::<F>),
                (&mut context as *mut QueryContext<'_, F>).cast(),
            )
        };
        resume_callback_panic(context.panic.take());
        stats.into()
    }

    pub fn query_closest<F>(
        &self,
        point: Vec3,
        mask_bits: u64,
        require_all_bits: bool,
        min_distance_sqr: f32,
        mut f: F,
    ) -> (QueryStats, f32)
    where
        F: FnMut(TreeClosestHit) -> f32,
    {
        assert_valid_vec3(point);
        assert!(!min_distance_sqr.is_nan() && min_distance_sqr >= 0.0);
        let mut min_distance_sqr = min_distance_sqr;
        let mut context = ClosestContext {
            f: &mut f,
            panic: None,
            tree_id: self.tree_id,
        };
        let stats = unsafe {
            sys::b3DynamicTree_QueryClosest(
                &self.raw,
                point.into(),
                mask_bits,
                require_all_bits,
                Some(closest_callback::<F>),
                (&mut context as *mut ClosestContext<'_, F>).cast(),
                &mut min_distance_sqr,
            )
        };
        resume_callback_panic(context.panic.take());
        (stats.into(), min_distance_sqr)
    }

    pub fn ray_cast<F>(
        &self,
        input: TreeRayCastInput,
        mask_bits: u64,
        require_all_bits: bool,
        mut f: F,
    ) -> QueryStats
    where
        F: FnMut(TreeCastHit<TreeRayCastInput>) -> f32,
    {
        let input = input.raw();
        let mut context = RayCastContext {
            f: &mut f,
            panic: None,
            tree_id: self.tree_id,
        };
        let stats = unsafe {
            sys::b3DynamicTree_RayCast(
                &self.raw,
                &input,
                mask_bits,
                require_all_bits,
                Some(ray_cast_callback::<F>),
                (&mut context as *mut RayCastContext<'_, F>).cast(),
            )
        };
        resume_callback_panic(context.panic.take());
        stats.into()
    }

    pub fn box_cast<F>(
        &self,
        input: TreeBoxCastInput,
        mask_bits: u64,
        require_all_bits: bool,
        mut f: F,
    ) -> QueryStats
    where
        F: FnMut(TreeCastHit<TreeBoxCastInput>) -> f32,
    {
        let input = input.raw();
        let mut context = BoxCastContext {
            f: &mut f,
            panic: None,
            tree_id: self.tree_id,
        };
        let stats = unsafe {
            sys::b3DynamicTree_BoxCast(
                &self.raw,
                &input,
                mask_bits,
                require_all_bits,
                Some(box_cast_callback::<F>),
                (&mut context as *mut BoxCastContext<'_, F>).cast(),
            )
        };
        resume_callback_panic(context.panic.take());
        stats.into()
    }

    pub fn validate(&self) {
        unsafe { sys::b3DynamicTree_Validate(&self.raw) };
    }

    pub fn validate_no_enlarged(&self) {
        unsafe { sys::b3DynamicTree_ValidateNoEnlarged(&self.raw) };
    }

    pub fn height(&self) -> i32 {
        unsafe { sys::b3DynamicTree_GetHeight(&self.raw) }
    }

    pub fn area_ratio(&self) -> f32 {
        unsafe { sys::b3DynamicTree_GetAreaRatio(&self.raw) }
    }

    pub fn root_bounds(&self) -> Aabb {
        unsafe { sys::b3DynamicTree_GetRootBounds(&self.raw) }.into()
    }

    pub fn proxy_count(&self) -> i32 {
        unsafe { sys::b3DynamicTree_GetProxyCount(&self.raw) }
    }

    pub fn rebuild(&mut self, full_build: bool) -> i32 {
        unsafe { sys::b3DynamicTree_Rebuild(&mut self.raw, full_build) }
    }

    pub fn byte_count(&self) -> i32 {
        unsafe { sys::b3DynamicTree_GetByteCount(&self.raw) }
    }

    pub fn save(&self, path: impl AsRef<Path>) -> Result<()> {
        let raw_path = path_to_cstring(&path)?;
        let file = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(path.as_ref())
            .map_err(|_| Error::InvalidInput)?;
        drop(file);
        unsafe { sys::b3DynamicTree_Save(&self.raw, raw_path.as_ptr()) };
        let metadata = std::fs::metadata(path.as_ref()).map_err(|_| Error::InvalidInput)?;
        if metadata.is_file() && metadata.len() > 0 {
            Ok(())
        } else {
            Err(Error::InvalidInput)
        }
    }

    pub fn load(path: impl AsRef<Path>, scale: f32) -> Result<Self> {
        assert!(scale.is_finite());
        let path = path_to_cstring(path)?;
        let raw = unsafe { sys::b3DynamicTree_Load(path.as_ptr(), scale) };
        if raw.version == 0 {
            Err(Error::InvalidInput)
        } else {
            let proxies = live_proxy_ids(&raw);
            Ok(Self {
                raw,
                proxies,
                tree_id: next_tree_id(),
            })
        }
    }

    fn proxy_id(&self, proxy: TreeProxy) -> Result<i32> {
        if proxy.tree_id == self.tree_id && self.proxies.contains(&proxy.id) {
            Ok(proxy.id)
        } else {
            Err(Error::InvalidInput)
        }
    }
}

impl Default for DynamicTree {
    fn default() -> Self {
        Self::new(16)
    }
}

impl Drop for DynamicTree {
    fn drop(&mut self) {
        unsafe { sys::b3DynamicTree_Destroy(&mut self.raw) };
    }
}

impl TreeRayCastInput {
    pub fn new(origin: Vec3, translation: Vec3, max_fraction: f32) -> Self {
        assert_valid_vec3(origin);
        assert_valid_vec3(translation);
        assert!((0.0..=1.0).contains(&max_fraction));
        Self {
            origin,
            translation,
            max_fraction,
        }
    }

    fn raw(self) -> sys::b3RayCastInput {
        Self::new(self.origin, self.translation, self.max_fraction);
        sys::b3RayCastInput {
            origin: self.origin.into(),
            translation: self.translation.into(),
            maxFraction: self.max_fraction,
        }
    }
}

impl From<sys::b3RayCastInput> for TreeRayCastInput {
    fn from(value: sys::b3RayCastInput) -> Self {
        Self {
            origin: value.origin.into(),
            translation: value.translation.into(),
            max_fraction: value.maxFraction,
        }
    }
}

impl TreeBoxCastInput {
    pub fn new(aabb: Aabb, translation: Vec3, max_fraction: f32) -> Self {
        assert_valid_aabb(aabb);
        assert_valid_vec3(translation);
        assert!((0.0..=1.0).contains(&max_fraction));
        Self {
            aabb,
            translation,
            max_fraction,
        }
    }

    fn raw(self) -> sys::b3BoxCastInput {
        Self::new(self.aabb, self.translation, self.max_fraction);
        sys::b3BoxCastInput {
            box_: self.aabb.into(),
            translation: self.translation.into(),
            maxFraction: self.max_fraction,
        }
    }
}

impl From<sys::b3BoxCastInput> for TreeBoxCastInput {
    fn from(value: sys::b3BoxCastInput) -> Self {
        Self {
            aabb: value.box_.into(),
            translation: value.translation.into(),
            max_fraction: value.maxFraction,
        }
    }
}

type CallbackPanic = Box<dyn Any + Send + 'static>;

struct QueryContext<'a, F> {
    f: &'a mut F,
    panic: Option<CallbackPanic>,
    tree_id: u64,
}

unsafe extern "C" fn query_callback<F>(proxy_id: i32, user_data: u64, context: *mut c_void) -> bool
where
    F: FnMut(TreeHit) -> bool,
{
    let context = unsafe { &mut *context.cast::<QueryContext<'_, F>>() };
    if context.panic.is_some() {
        return false;
    }

    match catch_unwind(AssertUnwindSafe(|| {
        (context.f)(TreeHit {
            proxy: TreeProxy {
                id: proxy_id,
                tree_id: context.tree_id,
            },
            user_data,
        })
    })) {
        Ok(keep_going) => keep_going,
        Err(panic) => {
            context.panic = Some(panic);
            false
        }
    }
}

struct ClosestContext<'a, F> {
    f: &'a mut F,
    panic: Option<CallbackPanic>,
    tree_id: u64,
}

unsafe extern "C" fn closest_callback<F>(
    distance_sqr_min: f32,
    proxy_id: i32,
    user_data: u64,
    context: *mut c_void,
) -> f32
where
    F: FnMut(TreeClosestHit) -> f32,
{
    let context = unsafe { &mut *context.cast::<ClosestContext<'_, F>>() };
    if context.panic.is_some() {
        return distance_sqr_min;
    }

    match catch_unwind(AssertUnwindSafe(|| {
        (context.f)(TreeClosestHit {
            distance_sqr_min,
            proxy: TreeProxy {
                id: proxy_id,
                tree_id: context.tree_id,
            },
            user_data,
        })
    })) {
        Ok(distance) => distance,
        Err(panic) => {
            context.panic = Some(panic);
            distance_sqr_min
        }
    }
}

struct RayCastContext<'a, F> {
    f: &'a mut F,
    panic: Option<CallbackPanic>,
    tree_id: u64,
}

unsafe extern "C" fn ray_cast_callback<F>(
    input: *const sys::b3RayCastInput,
    proxy_id: i32,
    user_data: u64,
    context: *mut c_void,
) -> f32
where
    F: FnMut(TreeCastHit<TreeRayCastInput>) -> f32,
{
    let context = unsafe { &mut *context.cast::<RayCastContext<'_, F>>() };
    if context.panic.is_some() {
        return 0.0;
    }

    match catch_unwind(AssertUnwindSafe(|| {
        (context.f)(TreeCastHit {
            input: unsafe { (*input).into() },
            proxy: TreeProxy {
                id: proxy_id,
                tree_id: context.tree_id,
            },
            user_data,
        })
    })) {
        Ok(fraction) => fraction,
        Err(panic) => {
            context.panic = Some(panic);
            0.0
        }
    }
}

struct BoxCastContext<'a, F> {
    f: &'a mut F,
    panic: Option<CallbackPanic>,
    tree_id: u64,
}

unsafe extern "C" fn box_cast_callback<F>(
    input: *const sys::b3BoxCastInput,
    proxy_id: i32,
    user_data: u64,
    context: *mut c_void,
) -> f32
where
    F: FnMut(TreeCastHit<TreeBoxCastInput>) -> f32,
{
    let context = unsafe { &mut *context.cast::<BoxCastContext<'_, F>>() };
    if context.panic.is_some() {
        return 0.0;
    }

    match catch_unwind(AssertUnwindSafe(|| {
        (context.f)(TreeCastHit {
            input: unsafe { (*input).into() },
            proxy: TreeProxy {
                id: proxy_id,
                tree_id: context.tree_id,
            },
            user_data,
        })
    })) {
        Ok(fraction) => fraction,
        Err(panic) => {
            context.panic = Some(panic);
            0.0
        }
    }
}

fn resume_callback_panic(panic: Option<CallbackPanic>) {
    if let Some(panic) = panic {
        resume_unwind(panic);
    }
}

fn assert_valid_aabb(aabb: Aabb) {
    assert_valid_vec3(aabb.lower_bound);
    assert_valid_vec3(aabb.upper_bound);
    assert!(aabb.lower_bound.x <= aabb.upper_bound.x);
    assert!(aabb.lower_bound.y <= aabb.upper_bound.y);
    assert!(aabb.lower_bound.z <= aabb.upper_bound.z);
}

fn assert_valid_vec3(value: Vec3) {
    assert!(value.x.is_finite() && value.y.is_finite() && value.z.is_finite());
}

fn path_to_cstring(path: impl AsRef<Path>) -> Result<CString> {
    let path = path.as_ref().to_str().ok_or(Error::InvalidInput)?;
    CString::new(path).map_err(|_| Error::InvalidInput)
}

fn live_proxy_ids(raw: &sys::b3DynamicTree) -> HashSet<i32> {
    let mut proxies = HashSet::new();
    for proxy_id in 0..raw.nodeCapacity {
        let node = unsafe { *raw.nodes.add(proxy_id as usize) };
        if node.flags & sys::b3TreeNodeFlags_b3_leafNode as u16 != 0 {
            proxies.insert(proxy_id);
        }
    }
    proxies
}

fn next_tree_id() -> u64 {
    static NEXT_TREE_ID: AtomicU64 = AtomicU64::new(1);
    NEXT_TREE_ID.fetch_add(1, Ordering::Relaxed)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        fs,
        time::{SystemTime, UNIX_EPOCH},
    };

    fn aabb(min: f32, max: f32) -> Aabb {
        Aabb {
            lower_bound: Vec3::new(min, min, min),
            upper_bound: Vec3::new(max, max, max),
        }
    }

    #[test]
    fn dynamic_tree_proxies_queries_casts_and_persistence_work() {
        let mut tree = DynamicTree::new(2);
        let proxy_a = tree.create_proxy(aabb(-1.0, 1.0), 0x1, 11);
        let proxy_b = tree.create_proxy(
            Aabb {
                lower_bound: Vec3::new(3.0, -1.0, -1.0),
                upper_bound: Vec3::new(4.0, 1.0, 1.0),
            },
            0x2,
            22,
        );

        assert_eq!(tree.proxy_count(), 2);
        assert_eq!(tree.user_data(proxy_a).unwrap(), 11);
        assert_eq!(tree.category_bits(proxy_b).unwrap(), 0x2);
        tree.set_category_bits(proxy_b, 0x4).unwrap();
        assert_eq!(tree.category_bits(proxy_b).unwrap(), 0x4);
        assert_eq!(tree.aabb(proxy_a).unwrap(), aabb(-1.0, 1.0));

        let mut hits = Vec::new();
        let stats = tree.query(aabb(-2.0, 2.0), u64::MAX, false, |hit| {
            hits.push(hit.user_data);
            true
        });
        assert_eq!(hits, vec![11]);
        assert!(stats.node_visits > 0);

        let mut stopped = 0;
        tree.query(aabb(-10.0, 10.0), 0x1, true, |_| {
            stopped += 1;
            false
        });
        assert_eq!(stopped, 1);

        let (_, min_sqr) = tree.query_closest(Vec3::ZERO, u64::MAX, false, f32::INFINITY, |hit| {
            assert_eq!(hit.user_data, 11);
            0.0
        });
        assert_eq!(min_sqr, 0.0);

        let mut ray_hits = Vec::new();
        let ray_stats = tree.ray_cast(
            TreeRayCastInput::new(Vec3::new(-5.0, 0.0, 0.0), Vec3::new(10.0, 0.0, 0.0), 1.0),
            u64::MAX,
            false,
            |hit| {
                ray_hits.push(hit.user_data);
                0.0_f32.min(hit.input.max_fraction)
            },
        );
        assert_eq!(ray_hits.len(), 1);
        assert_eq!(ray_stats.leaf_visits, 1);

        let mut box_hits = Vec::new();
        let box_stats = tree.box_cast(
            TreeBoxCastInput::new(aabb(-0.25, 0.25), Vec3::new(4.0, 0.0, 0.0), 1.0),
            u64::MAX,
            false,
            |hit| {
                box_hits.push(hit.user_data);
                0.0_f32.min(hit.input.max_fraction)
            },
        );
        assert_eq!(box_hits.len(), 1);
        assert_eq!(box_stats.leaf_visits, 1);

        tree.move_proxy(proxy_a, aabb(5.0, 6.0)).unwrap();
        assert_eq!(tree.aabb(proxy_a).unwrap(), aabb(5.0, 6.0));
        tree.enlarge_proxy(
            proxy_a,
            Aabb {
                lower_bound: Vec3::new(4.0, 4.0, 4.0),
                upper_bound: Vec3::new(7.0, 7.0, 7.0),
            },
        )
        .unwrap();
        assert_eq!(tree.rebuild(true), 2);
        tree.validate();
        tree.validate_no_enlarged();
        assert!(tree.height() >= 0);
        assert!(tree.area_ratio().is_finite());
        assert!(tree.byte_count() > 0);
        assert!(tree.root_bounds().lower_bound.x <= tree.root_bounds().upper_bound.x);

        let path = tree_path();
        tree.save(&path).unwrap();
        let loaded = DynamicTree::load(&path, 1.0).unwrap();
        fs::remove_file(&path).unwrap();
        assert_eq!(loaded.proxy_count(), 2);
        let loaded_proxy_b = loaded.proxy_from_id(proxy_b.id()).unwrap();
        assert_eq!(loaded.user_data(loaded_proxy_b).unwrap(), 22);

        tree.destroy_proxy(proxy_a).unwrap();
        assert_eq!(tree.user_data(proxy_a).err(), Some(Error::InvalidInput));
        tree.destroy_proxy(proxy_b).unwrap();
        assert_eq!(tree.proxy_count(), 0);
    }

    #[test]
    fn dynamic_tree_rejects_foreign_and_destroyed_proxies() {
        let mut tree_a = DynamicTree::new(1);
        let mut tree_b = DynamicTree::new(1);
        let proxy_a = tree_a.create_proxy(aabb(-1.0, 1.0), 0x1, 11);
        let proxy_b = tree_b.create_proxy(aabb(-1.0, 1.0), 0x1, 22);

        assert_eq!(tree_a.user_data(proxy_b).err(), Some(Error::InvalidInput));
        tree_a.destroy_proxy(proxy_a).unwrap();
        assert_eq!(tree_a.user_data(proxy_a).err(), Some(Error::InvalidInput));
        assert_eq!(
            tree_a.destroy_proxy(proxy_a).err(),
            Some(Error::InvalidInput)
        );
    }

    #[test]
    fn dynamic_tree_callbacks_resume_panics() {
        let mut tree = DynamicTree::new(1);
        tree.create_proxy(aabb(-1.0, 1.0), 0x1, 11);

        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            tree.query(aabb(-2.0, 2.0), u64::MAX, false, |_| panic!("boom"));
        }));

        assert!(result.is_err());
    }

    #[test]
    fn dynamic_tree_save_directory_returns_error() {
        let tree = DynamicTree::new(1);

        assert_eq!(
            tree.save(std::env::temp_dir()).err(),
            Some(Error::InvalidInput)
        );
    }

    fn tree_path() -> std::path::PathBuf {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        std::env::temp_dir().join(format!("box3d-tree-{nanos}.bin"))
    }
}
