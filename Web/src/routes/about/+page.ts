export const prerender = false;
export const csr = true;

export const load = async ({ fetch }) => {
    const res = await fetch('/api/about');
    if (res.status === 200) {
        const data = await res.json();
        return { ...data };
    }
    return {};
};

